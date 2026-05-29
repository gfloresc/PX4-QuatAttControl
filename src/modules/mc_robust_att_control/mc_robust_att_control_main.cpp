#include "mc_robust_att_control.hpp"
#include <px4_platform_common/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/parameter_update.h>

// ---------------------------------------------------------------------------
// Constructor / Destructor / Init
// ---------------------------------------------------------------------------

McRobustAttControl::McRobustAttControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	parameters_update(true);
}

McRobustAttControl::~McRobustAttControl()
{
	perf_free(_loop_perf);
}

bool McRobustAttControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("gyro callback registration failed");
		return false;
	}
	return true;
}

// ---------------------------------------------------------------------------
// Parameter helpers
// ---------------------------------------------------------------------------

void McRobustAttControl::update_inertia_matrix()
{
	_J.setZero();
	_J(0, 0) = math::max(_param_jxx.get(), 1e-6f);
	_J(1, 1) = math::max(_param_jyy.get(), 1e-6f);
	_J(2, 2) = math::max(_param_jzz.get(), 1e-6f);
}

void McRobustAttControl::reset_filters(float sample_rate_hz)
{
	// Yaw torque LPF
	const float ycut = math::max(_param_yaw_cutoff.get(), 0.1f);
	_yaw_torque_lpf.set_cutoff_frequency(sample_rate_hz, ycut);
	_yaw_torque_lpf.reset(0.f);

	// Omegad_dot LPF — cutoff from parameter MC_RATT_ODCUT
	const float odcut = math::max(_param_odd_cutoff.get(), 0.1f);
	_odd_lpf_x.set_cutoff_frequency(sample_rate_hz, odcut);
	_odd_lpf_y.set_cutoff_frequency(sample_rate_hz, odcut);
	_odd_lpf_z.set_cutoff_frequency(sample_rate_hz, odcut);
	_odd_lpf_x.reset(0.f);
	_odd_lpf_y.reset(0.f);
	_odd_lpf_z.reset(0.f);
}

void McRobustAttControl::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s p{};
		_parameter_update_sub.copy(&p);
		updateParams();
		update_inertia_matrix();
		reset_filters(1.f / math::max(_dt, 1e-3f));
	}
}

// ---------------------------------------------------------------------------
// Subscription update
// ---------------------------------------------------------------------------

bool McRobustAttControl::update_subscriptions()
{
	(void)_vehicle_attitude_sub.update(&_att);
	(void)_vehicle_attitude_setpoint_sub.update(&_att_sp);
	(void)_vehicle_control_mode_sub.update(&_control_mode);
	(void)_vehicle_land_detected_sub.update(&_landed);
	(void)_vehicle_status_sub.update(&_vehicle_status);
	(void)_battery_status_sub.update(&_battery_status);
	(void)_control_allocator_status_sub.update(&_allocator_status);

	if (_param_bat_scale_en.get() > 0 &&
	    PX4_ISFINITE(_battery_status.scale) &&
	    _battery_status.scale > 0.f) {
		_battery_scale = _battery_status.scale;
	} else {
		_battery_scale = 1.f;
	}

	return PX4_ISFINITE(_att.q[0]) && PX4_ISFINITE(_att_sp.q_d[0]);
}

// ---------------------------------------------------------------------------
// Attitude error  (PX4 convention: qe = canonical(q^{-1} * qd))
// Note: this is the conjugate of the paper's Qe = Qd^{-1} * Q,
//       so Re_code = Dcmf(qe) satisfies Re_code^T = Re_paper.
//       All feedforward terms use Re_code.transpose() to match the paper.
// ---------------------------------------------------------------------------

Quatf McRobustAttControl::attitude_error_px4(const Quatf &q, const Quatf &qd) const
{
	return (q.inversed() * qd).canonical();
}

// ---------------------------------------------------------------------------
// Finite-difference estimate of Omegad and Omegad_dot
//
// Quaternion kinematics:  Qd_dot = 0.5 * Qd * (0, Omegad)
// => (0, Omegad) = 2 * Qd^{-1} * Qd_dot
//
// We approximate Qd_dot ~ (qd - qd_prev) / dt, ensuring the two quaternions
// are in the same hemisphere (dot product >= 0) to avoid sign-flip artefacts.
// Omegad_dot is then the finite difference of Omegad, low-pass filtered.
// ---------------------------------------------------------------------------

void McRobustAttControl::compute_omega_d(const Quatf &qd,
                                          Vector3f    &omega_d_out,
                                          Vector3f    &omega_d_dot_out)
{
    if (!_prev_valid || _dt < 1e-5f) {
        _qd_prev      = qd;
        _omega_d_prev.setZero();
        _prev_valid   = true;
        omega_d_out.setZero();
        omega_d_dot_out.setZero();
        return;
    }

    // Ensure same hemisphere
    Quatf qd_curr = qd;
    const float dot = qd_curr(0)*_qd_prev(0) + qd_curr(1)*_qd_prev(1)
                    + qd_curr(2)*_qd_prev(2) + qd_curr(3)*_qd_prev(3);
    if (dot < 0.f) {
        qd_curr = Quatf(-qd_curr(0), -qd_curr(1),
                        -qd_curr(2), -qd_curr(3));
    }

    const float inv_dt = 1.f / _dt;

    // Omegad via finite difference of qd
    Quatf qd_dot{
        (qd_curr(0) - _qd_prev(0)) * inv_dt,
        (qd_curr(1) - _qd_prev(1)) * inv_dt,
        (qd_curr(2) - _qd_prev(2)) * inv_dt,
        (qd_curr(3) - _qd_prev(3)) * inv_dt
    };
    Quatf omega_d_quat = qd_curr.inversed() * qd_dot;
    Vector3f omega_d{
        2.f * omega_d_quat(1),
        2.f * omega_d_quat(2),
        2.f * omega_d_quat(3)
    };

    // Clamp omega_d to avoid spikes (qd rarely changes faster than 5 rad/s)
    const float od_max = 10.f;
    for (int i = 0; i < 3; i++) {
        omega_d(i) = math::constrain(omega_d(i), -od_max, od_max);
    }

    // Omegad_dot — ONLY compute if omega_d_prev is already valid
    // On first valid omega_d, initialize prev and return zero dot
    if (!_omega_d_prev_valid) {
        _omega_d_prev       = omega_d;
        _omega_d_prev_valid = true;
        omega_d_out     = omega_d;
        omega_d_dot_out.setZero();
        _qd_prev = qd_curr;
        return;
    }

    const Vector3f odd_raw = (omega_d - _omega_d_prev) * inv_dt;

    // Clamp derivative before filtering
    Vector3f odd_clamped{};
    const float odd_max = 200.f;  // rad/s^2
    for (int i = 0; i < 3; i++) {
        odd_clamped(i) = math::constrain(odd_raw(i), -odd_max, odd_max);
    }

    Vector3f omega_d_dot{
        _odd_lpf_x.apply(odd_clamped(0)),
        _odd_lpf_y.apply(odd_clamped(1)),
        _odd_lpf_z.apply(odd_clamped(2))
    };

    _qd_prev      = qd_curr;
    _omega_d_prev = omega_d;

    omega_d_out     = omega_d;
    omega_d_dot_out = omega_d_dot;
}

// ---------------------------------------------------------------------------
// Tracking torque — Theorem 2 (paper equation 43)
//
// J^{-1} tau = J^{-1}(Omega x J*Omega)
//            - Omega_hat * Re^T * Omegad        (feedforward Coriolis cancel)
//            + Re^T * Omegad_dot                (feedforward inertia cancel)
//            - k1 * diag(|Omega_tilde|) * qve   (nonlinear coupling)
//            - kappa * Omega_tilde * sqrt(1-q0e) (nonlinear damping)
//            - k3 * qve                          (attitude restoring)
//            - k4 * Omega_tilde                  (linear damping)
//
// PX4 convention note: Re_code = Dcmf(qe) = Re_paper^T,
//   so Re_paper^T = Re_code  =>  the code uses Re directly (not transposed)
//   for the feedforward terms.
// ---------------------------------------------------------------------------

Vector3f McRobustAttControl::compute_torque_theorem2(const Quatf    &qe,
                                                      const Vector3f &omega,
                                                      const Vector3f &omega_d,
                                                      const Vector3f &omega_d_dot) const
{
	const float    q0e = qe(0);
	const Vector3f qve{qe(1), qe(2), qe(3)};

	const float k1    = _param_k1.get();
	const float k3    = _param_k3.get();
	const float k4    = _param_k4.get();
	const float kappa = _param_kappa.get();

	// Re in PX4 convention: Re_code^T == Re_paper
	// So Re_paper^T * Omegad  =  Re_code * Omegad
	const Dcmf   Re_code(qe);
	const Vector3f Re_T_omegad     = Re_code * omega_d;      // Re_paper^T * Omegad
	const Vector3f Re_T_omegadot   = Re_code * omega_d_dot;  // Re_paper^T * Omegad_dot

	// Angular velocity error:  Omega_tilde = Omega - Re_paper^T * Omegad
	const Vector3f omega_tilde = omega - Re_T_omegad;

	// Gyroscopic compensation: Omega x (J * Omega)
	const Vector3f Jomega = _J * omega;
	const Vector3f gyro   = omega.cross(Jomega);

	// Feedforward Coriolis cancellation: -Omega_hat * Re^T * Omegad
	//   = -(Omega x Re_T_omegad)
	const Vector3f ff_coriolis = -(omega.cross(Re_T_omegad));

	// Feedforward inertia cancellation: Re^T * Omegad_dot
	const Vector3f ff_inertia = Re_T_omegadot;
	//const Vector3f ff_inertia{};  // deshabilitado temporalmente

	// Nonlinear coupling: -k1 * diag(|Omega_tilde|) * qve
	Vector3f diag_abs_wt_qve{};
	diag_abs_wt_qve(0) = fabsf(omega_tilde(0)) * qve(0);
	diag_abs_wt_qve(1) = fabsf(omega_tilde(1)) * qve(1);
	diag_abs_wt_qve(2) = fabsf(omega_tilde(2)) * qve(2);

	const float root_term = sqrtf(math::max(0.f, 1.f - q0e));

	// Feedback shaping (all terms use Omega_tilde, not raw Omega)
	// Feedback shaping — signos positivos en k1 y k3 porque
	// qve_code = -qve_paper (convención PX4 invertida respecto al paper)
	const Vector3f shaping = + k1 * diag_abs_wt_qve    // POSITIVO
		                 + k3 * qve                 // POSITIVO
		                 - kappa * root_term * omega_tilde
		                 - k4 * omega_tilde;

	// Full torque command:
	//   tau = Omega x J*Omega  +  J * (ff_coriolis + ff_inertia + shaping)
	return gyro + _J * (ff_coriolis + ff_inertia + shaping);
}

// ---------------------------------------------------------------------------
// Torque scaling and output filtering
// ---------------------------------------------------------------------------

Vector3f McRobustAttControl::scale_and_constrain_torque(const Vector3f &tau_raw)
{
	Vector3f tau = tau_raw;

	const Vector3f limits{
		math::max(_param_tau_max_x.get(), 1e-4f),
		math::max(_param_tau_max_y.get(), 1e-4f),
		math::max(_param_tau_max_z.get(), 1e-4f)
	};

	// Direction-preserving global scaling
	float alpha = 1.f;
	for (int i = 0; i < 3; i++) {
		const float abs_tau = fabsf(tau(i));
		if (abs_tau > limits(i)) {
			alpha = math::min(alpha, limits(i) / abs_tau);
		}
	}
	tau *= alpha;

	// Per-axis hard clamp
	for (int i = 0; i < 3; i++) {
		tau(i) = math::constrain(tau(i), -limits(i), limits(i));
	}

	// Battery scaling
	tau *= _battery_scale;

	// Optional yaw torque LPF
	if (_param_yaw_cutoff.get() > 0.f) {
		tau(2) = _yaw_torque_lpf.apply(tau(2));
	}

	return tau;
}

// ---------------------------------------------------------------------------
// Publication helpers
// ---------------------------------------------------------------------------

void McRobustAttControl::publish_torque_and_thrust(const Vector3f &tau,
                                                    const Vector3f &thrust_body,
                                                    hrt_abstime     timestamp_sample)
{
	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp_sample = timestamp_sample;
	torque_sp.timestamp        = hrt_absolute_time();
	torque_sp.xyz[0] = PX4_ISFINITE(tau(0)) ? tau(0) : 0.f;
	torque_sp.xyz[1] = PX4_ISFINITE(tau(1)) ? tau(1) : 0.f;
	torque_sp.xyz[2] = PX4_ISFINITE(tau(2)) ? tau(2) : 0.f;
	_vehicle_torque_setpoint_pub.publish(torque_sp);

	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp_sample = timestamp_sample;
	thrust_sp.timestamp        = hrt_absolute_time();
	thrust_sp.xyz[0] = thrust_body(0) * _battery_scale;
	thrust_sp.xyz[1] = thrust_body(1) * _battery_scale;
	thrust_sp.xyz[2] = thrust_body(2) * _battery_scale;
	_vehicle_thrust_setpoint_pub.publish(thrust_sp);
}

void McRobustAttControl::publish_zero_torque(const Vector3f &thrust_body,
                                              hrt_abstime     timestamp_sample)
{
	publish_torque_and_thrust(Vector3f{}, thrust_body, timestamp_sample);
}

// ---------------------------------------------------------------------------
// Main control loop
// ---------------------------------------------------------------------------

void McRobustAttControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	parameters_update();

	vehicle_angular_velocity_s angular_velocity{};
	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = angular_velocity.timestamp_sample;
	if (_last_run == 0) { _last_run = now; }

	_dt      = math::constrain((now - _last_run) * 1e-6f, 0.000125f, 0.02f);
	_last_run = now;

	// Update LPF cutoffs dynamically (rate may vary slightly)
	const float sr = 1.f / _dt;
	if (_param_yaw_cutoff.get() > 0.f) {
		_yaw_torque_lpf.set_cutoff_frequency(sr, _param_yaw_cutoff.get());
	}
	const float odcut = math::max(_param_odd_cutoff.get(), 0.1f);
	_odd_lpf_x.set_cutoff_frequency(sr, odcut);
	_odd_lpf_y.set_cutoff_frequency(sr, odcut);
	_odd_lpf_z.set_cutoff_frequency(sr, odcut);

	if (!update_subscriptions()) {
		perf_end(_loop_perf);
		return;
	}

	const Vector3f thrust_body{
		_att_sp.thrust_body[0],
		_att_sp.thrust_body[1],
		_att_sp.thrust_body[2]
	};

	if (!_control_mode.flag_control_attitude_enabled ||
	    !_control_mode.flag_control_rates_enabled) {
		publish_zero_torque(thrust_body, angular_velocity.timestamp_sample);
		perf_end(_loop_perf);
		return;
	}

	// Normalize attitude and setpoint quaternions
	Quatf q(_att.q);
	q.normalize();

	// --- Trajectory selector ---
	Quatf qd;
	const bool traj_active = (_param_traj_en.get() == 1);

	if (traj_active) {
	    	// Detect rising edge — record start time
		if (!_traj_was_active) {
		    _traj_start_time    = now;
		    _traj_was_active    = true;
		    _prev_valid         = false;
		    _omega_d_prev_valid = false;
		    _qd_prev            = Quatf(_att_sp.q_d);
		    _odd_lpf_x.reset(0.f);
		    _odd_lpf_y.reset(0.f);
		    _odd_lpf_z.reset(0.f);
		    _omega_d_prev.setZero();

		    // Capturar yaw solo si nunca se ha capturado
			if (!_traj_yaw_initialized) {
			    const matrix::Eulerf euler_init(matrix::Quatf(_att.q));
			    _traj_yaw = euler_init.psi();
			    _traj_yaw_initialized = true;
			}
		    PX4_INFO("Trajectory activated");
		}

		// Time since trajectory start [s]
		const float t = (now - _traj_start_time) * 1e-6f;
		const float window = 0.5f * (1.f - cosf(M_PI_F * math::min(t / _param_traj_dur.get(), 1.0f)));
		//const float px = window * 0.5f * sinf(1.6f * t);
		//const float py = window * 0.25f * sinf(3.2f * t);
		// Roll y pitch a frecuencias que yaw puede compensar
		const float px = window * 0.3f * sinf(0.8f * t);   // roll ±17° a 0.8 rad/s
		const float py = window * 0.3f * sinf(1.2f * t);   // pitch ±17° a 1.2 rad/s
		
		//Quatf qz_yaw(cosf(_traj_yaw * 0.5f), 0.f, 0.f, sinf(_traj_yaw * 0.5f));
		//Quatf qx(cosf(px * 0.5f), sinf(px * 0.5f), 0.f, 0.f);
		//Quatf qy(cosf(py * 0.5f), 0.f, sinf(py * 0.5f), 0.f);
		//qd = (qz_yaw * qx * qy).normalized();
		
		
		const float cpsi   = cosf(_traj_yaw * 0.5f);
		const float spsi   = sinf(_traj_yaw * 0.5f);
		const float cphi   = cosf(px * 0.5f);
		const float sphi   = sinf(px * 0.5f);
		const float ctheta = cosf(py * 0.5f);
		const float stheta = sinf(py * 0.5f);

		qd(0) = cpsi*cphi*ctheta + spsi*sphi*stheta;
		qd(1) = cpsi*sphi*ctheta - spsi*cphi*stheta;
		qd(2) = cpsi*cphi*stheta + spsi*sphi*ctheta;
		qd(3) = spsi*cphi*ctheta - cpsi*sphi*stheta;
		qd = qd.normalized();
	    
		// Auto-deactivate after duration (0 = infinite)
		//const float dur = _param_traj_dur.get();
		//if (dur > 0.f && t >= dur) {
		//    _param_traj_en.set(0);
		//    _param_traj_en.commit();
		//    PX4_INFO("Trajectory finished after %.1f s", (double)dur);
		//}

	} else {
	    // Normal PX4 setpoint
	    if (_traj_was_active) {
		_traj_was_active    = false;
		_prev_valid         = false;  // reset omega_d estimator
		_omega_d_prev_valid = false;
		PX4_INFO("Trajectory deactivated");
	    }
	    qd = Quatf(_att_sp.q_d);
	    qd.normalize();
	}

	// Attitude error
	const Quatf qe = attitude_error_px4(q, qd);

	// Angular velocity (body frame)
	const Vector3f omega{
		angular_velocity.xyz[0],
		angular_velocity.xyz[1],
		angular_velocity.xyz[2]
	};

	// Desired angular velocity and its derivative (finite-difference + LPF)
	Vector3f omega_d{};
	Vector3f omega_d_dot{};
	compute_omega_d(qd, omega_d, omega_d_dot);
	
	// Publicar qd interno para que quede en el log
	if (traj_active) {
	    vehicle_attitude_setpoint_s att_sp_log{};
	    att_sp_log.timestamp = hrt_absolute_time();
	    // quita esta línea: att_sp_log.timestamp_sample = now;
	    att_sp_log.q_d[0] = qd(0);
	    att_sp_log.q_d[1] = qd(1);
	    att_sp_log.q_d[2] = qd(2);
	    att_sp_log.q_d[3] = qd(3);
	    att_sp_log.thrust_body[0] = _att_sp.thrust_body[0];
	    att_sp_log.thrust_body[1] = _att_sp.thrust_body[1];
	    att_sp_log.thrust_body[2] = _att_sp.thrust_body[2];
	    _vehicle_attitude_setpoint_pub.publish(att_sp_log);
	}

	// Compute and publish torque
	//Vector3f tau_raw = (qe, omega, omega_d, omega_d_dot);
	Vector3f tau_raw = compute_torque_theorem2(qe, omega, omega_d, omega_d_dot);
	Vector3f tau_cmd = scale_and_constrain_torque(tau_raw);

	publish_torque_and_thrust(tau_cmd, thrust_body, angular_velocity.timestamp_sample);
	perf_end(_loop_perf);
}

// ---------------------------------------------------------------------------
// Module boilerplate
// ---------------------------------------------------------------------------

int McRobustAttControl::task_spawn(int argc, char *argv[])
{
	McRobustAttControl *instance = new McRobustAttControl();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
	}
	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

McRobustAttControl *McRobustAttControl::instantiate(int argc, char *argv[])
{
	(void)argc; (void)argv;
	return new McRobustAttControl();
}

int McRobustAttControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int McRobustAttControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Multicopter robust attitude controller — Theorem 2 (Tracking).

Implements the full tracking control law from:
  "Almost Global Exponential Attitude Stabilization via Continuous
   Nonlinear Quaternion Feedback" (Flores & Torres).

Control law (body frame):
  tau = Omega x J*Omega
      + J * [ -Omega x (Re^T Omegad)   (feedforward Coriolis)
              + Re^T * Omegad_dot       (feedforward inertia)
              - k1*diag(|Omega_tilde|)*qve
              - kappa*Omega_tilde*sqrt(1-q0e)
              - k3*qve
              - k4*Omega_tilde ]

where Omega_tilde = Omega - Re^T * Omegad is the velocity error,
and Omegad / Omegad_dot are estimated by finite-differencing q_d.
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_robust_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_robust_att_control_main(int argc, char *argv[])
{
	return McRobustAttControl::main(argc, argv);
}
