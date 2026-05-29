#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/workqueue.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>

using matrix::Dcmf;
using matrix::Quatf;
using matrix::SquareMatrix3f;
using matrix::Vector3f;

class McRobustAttControl : public ModuleBase<McRobustAttControl>,
                           public ModuleParams,
                           public px4::WorkItem
{
public:
	McRobustAttControl();
	~McRobustAttControl() override;

	static int task_spawn(int argc, char *argv[]);
	static McRobustAttControl *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_update(bool force = false);
	bool update_subscriptions();
	void update_inertia_matrix();
	void reset_filters(float sample_rate_hz);

	// Attitude error: qe = canonical(q^{-1} * qd)
	// Note: this is the inverse of the paper convention Qe = Qd^{-1} * Q,
	// so Re_code = R(qe) satisfies Re_code^T = Re_paper.
	Quatf attitude_error_px4(const Quatf &q, const Quatf &qd) const;

	// Compute desired angular velocity Omegad from finite difference of qd,
	// and its filtered derivative Omegad_dot.
	// Both are expressed in the body frame of the desired attitude.
	void compute_omega_d(const Quatf &qd,
	                     Vector3f &omega_d_out,
	                     Vector3f &omega_d_dot_out);

	// Full tracking torque — Theorem 2.
	// omega_tilde = omega - Re^T * omega_d  (velocity error in body frame)
	// Re = Dcmf(qe) where qe uses PX4 convention above.
	Vector3f compute_torque_theorem2(const Quatf &qe,
	                                 const Vector3f &omega,
	                                 const Vector3f &omega_d,
	                                 const Vector3f &omega_d_dot) const;

	Vector3f scale_and_constrain_torque(const Vector3f &tau_raw);

	void publish_torque_and_thrust(const Vector3f &tau,
	                               const Vector3f &thrust_body,
	                               hrt_abstime timestamp_sample);
	void publish_zero_torque(const Vector3f &thrust_body,
	                         hrt_abstime timestamp_sample);

	// ---------- subscriptions ----------
	uORB::SubscriptionInterval           _parameter_update_sub{ORB_ID(parameter_update), 1000};
	uORB::SubscriptionCallbackWorkItem   _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription                   _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription                   _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription                   _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription                   _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription                   _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription                   _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription                   _control_allocator_status_sub{ORB_ID(control_allocator_status)};

	// ---------- publications ----------
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	// ---------- cached uORB messages ----------
	vehicle_attitude_s            _att{};
	vehicle_attitude_setpoint_s   _att_sp{};
	vehicle_control_mode_s        _control_mode{};
	vehicle_land_detected_s       _landed{};
	vehicle_status_s              _vehicle_status{};
	battery_status_s              _battery_status{};
	control_allocator_status_s    _allocator_status{};

	// ---------- controller state ----------
	SquareMatrix3f _J{};
	float          _dt{0.001f};
	hrt_abstime    _last_run{0};
	float          _battery_scale{1.f};
	hrt_abstime    _traj_start_time{0};  // timestamp cuando se activo la trayectoria
	bool           _traj_was_active{false}; // para detectar flanco de activacion
	float 	       _traj_yaw{0.f};  // yaw capturado al inicio de la trayectoria
	bool 	       _traj_yaw_initialized{false};

	// State for finite-difference computation of Omegad and Omegad_dot.
	Quatf   _qd_prev{1.f, 0.f, 0.f, 0.f};
	Vector3f _omega_d_prev{};          // Omegad from previous cycle (unfiltered)
	bool    _prev_valid{false};        // true after first cycle with valid qd
	bool _omega_d_prev_valid{false};  // después de _prev_valid

	// Low-pass filters:
	//   - yaw torque LPF (scalar, same as before)
	//   - three scalar LPFs for the Omegad_dot components (noise from double diff)
	mutable math::LowPassFilter2p<float> _yaw_torque_lpf{250.f, 30.f};
	mutable math::LowPassFilter2p<float> _odd_lpf_x{250.f, 20.f};
	mutable math::LowPassFilter2p<float> _odd_lpf_y{250.f, 20.f};
	mutable math::LowPassFilter2p<float> _odd_lpf_z{250.f, 20.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_RATT_K1>)     _param_k1,
		(ParamFloat<px4::params::MC_RATT_K3>)     _param_k3,
		(ParamFloat<px4::params::MC_RATT_K4>)     _param_k4,
		(ParamFloat<px4::params::MC_RATT_KAP>)    _param_kappa,
		(ParamFloat<px4::params::MC_RATT_JXX>)    _param_jxx,
		(ParamFloat<px4::params::MC_RATT_JYY>)    _param_jyy,
		(ParamFloat<px4::params::MC_RATT_JZZ>)    _param_jzz,
		(ParamFloat<px4::params::MC_RATT_TMAX_X>) _param_tau_max_x,
		(ParamFloat<px4::params::MC_RATT_TMAX_Y>) _param_tau_max_y,
		(ParamFloat<px4::params::MC_RATT_TMAX_Z>) _param_tau_max_z,
		(ParamFloat<px4::params::MC_RATT_YCUT>)   _param_yaw_cutoff,
		(ParamFloat<px4::params::MC_RATT_ODCUT>)  _param_odd_cutoff,
		(ParamInt<px4::params::MC_RATT_BSCALE>)   _param_bat_scale_en,
		(ParamInt<px4::params::MC_RATT_TRAJ>)	  _param_traj_en,
		(ParamFloat<px4::params::MC_RATT_TDUR>)   _param_traj_dur
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
