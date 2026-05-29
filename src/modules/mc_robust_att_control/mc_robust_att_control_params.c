/**
 * @file mc_robust_att_control_params.c
 * Parameters for the robust quaternion attitude controller (Theorem 2 - Tracking).
 */

/**
 * Use robust attitude controller instead of default.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0
 * @max 1
 * @value 0 Default PX4 controller
 * @value 1 Robust tracking controller (Theorem 2)
 */
PARAM_DEFINE_INT32(MC_RATT_USE, 1);

/**
 * Robust attitude control gain k1.
 *
 * Nonlinear coupling gain. Set to 0 to disable the diag(|Omega_tilde|)*qve term.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 50.0
 * @decimal 4
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(MC_RATT_K1, 5.0f);

/**
 * Robust attitude control gain k3.
 *
 * Attitude restoring gain (stiffness).
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 50.0
 * @decimal 4
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(MC_RATT_K3, 15.0f);

/**
 * Robust attitude control gain k4.
 *
 * Linear velocity error damping gain.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 50.0
 * @decimal 4
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(MC_RATT_K4, 5.0f);

/**
 * Robust nonlinear damping gain kappa.
 *
 * Multiplies Omega_tilde * sqrt(1 - q0e).
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 50.0
 * @decimal 4
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(MC_RATT_KAP, 0.8f);

/**
 * Nominal inertia Jxx.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0001
 * @max 1.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(MC_RATT_JXX, 0.029125f);

/**
 * Nominal inertia Jyy.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0001
 * @max 1.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(MC_RATT_JYY, 0.029125f);

/**
 * Nominal inertia Jzz.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0001
 * @max 1.0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(MC_RATT_JZZ, 0.055225f);

/**
 * Max roll torque magnitude.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(MC_RATT_TMAX_X, 0.50f);

/**
 * Max pitch torque magnitude.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(MC_RATT_TMAX_Y, 0.50f);

/**
 * Max yaw torque magnitude.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(MC_RATT_TMAX_Z, 0.40f);

/**
 * Yaw torque low-pass cutoff frequency (Hz). Set 0 to disable.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(MC_RATT_YCUT, 20.0f);

/**
 * Omegad_dot low-pass filter cutoff frequency (Hz).
 *
 * Low-pass filter applied to the finite-difference estimate of the
 * desired angular acceleration Omegad_dot. Lower values reduce noise
 * at the cost of added phase lag in the feedforward term.
 * Recommended range: 5 to 30 Hz.
 *
 * @group Multicopter Robust Attitude Control
 * @min 1.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(MC_RATT_ODCUT, 10.0f);

/**
 * Enable internal attitude trajectory (paper Section V).
 *
 * When 1, ignores qd from PX4 and tracks the internal trajectory
 * qd(t) = Qz(pi/2 * sin(1.6t)) * Qy(pi/4 * sin(3.2t)).
 * Activate only when already hovering stably.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0
 * @max 1
 * @value 0 Use PX4 attitude setpoint
 * @value 1 Use internal paper trajectory
 */
PARAM_DEFINE_INT32(MC_RATT_TRAJ, 0);

/**
 * Internal trajectory duration in seconds.
 *
 * After this time, trajectory deactivates automatically and
 * the controller returns to holding the final attitude.
 * Set to 0 for infinite duration.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0.0
 * @max 60.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(MC_RATT_TDUR, 5.0f);

/**
 * Enable battery voltage scaling of torque/thrust.
 *
 * @group Multicopter Robust Attitude Control
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(MC_RATT_BSCALE, 0);
