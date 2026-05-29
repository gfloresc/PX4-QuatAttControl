# PX4-QuatAttControl

Robust quaternion-based attitude controller for PX4 multicopters, implementing **Theorem 2 (Tracking)** from:

> G. Flores and J. Torres, *"Almost Global Exponential Attitude Stabilization via Continuous Nonlinear Quaternion Feedback"*, TAMIU RAPTOR Lab.

The controller replaces the default `mc_att_control + mc_rate_control` pipeline with a direct torque law designed entirely in the unit quaternion space, providing semi-global exponential stability with an explicit decay rate and no sign-function discontinuities.

---

## Control Law

The full tracking torque (body frame) is:

```
τ = Ω × JΩ
  + J [ −Ω̂(Rₑᵀ Ωd)          (feedforward Coriolis)
        + Rₑᵀ Ω̇d              (feedforward inertia)
        − k1·diag(|Ω̃|)·qve   (nonlinear coupling)
        − κ·Ω̃·√(1−q0e)       (nonlinear damping)
        − k3·qve               (attitude restoring)
        − k4·Ω̃ ]              (linear damping)
```

where `Ω̃ = Ω − Rₑᵀ Ωd` is the angular velocity error and `Ωd`, `Ω̇d` are estimated by finite-differencing `qd`.

---

## Requirements

- Ubuntu 20.04 or 22.04
- ROS 2 (optional, not required for SITL)
- Gazebo Classic 11
- Python 3.8+

---

## Installation

### 1. Clone this repository

```bash
git clone https://github.com/gfloresc/PX4-QuatAttControl.git
cd PX4-QuatAttControl
```

### 2. Install PX4 dependencies

```bash
bash Tools/setup/ubuntu.sh
```

### 3. Install Python dependencies

```bash
pip3 install --user pyulog pymavlink
```

### 4. Build for SITL

```bash
make px4_sitl gazebo
```

This compiles the firmware including the robust attitude controller module (`mc_robust_att_control`).

---

## Running the Simulation

### 1. Launch Gazebo SITL

```bash
source /usr/share/gazebo/setup.bash
make px4_sitl gazebo
```

### 2. Launch QGroundControl (separate terminal)

```bash
~/QGroundControl.AppImage
```

Download QGroundControl from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

### 3. Enable virtual joystick in QGC

Go to **Application Settings → Virtual Joystick** and enable it.

---

## Controller Selection

The controller is selected via the `MC_RATT_USE` parameter:

| Value | Controller |
|-------|-----------|
| `0` | Default PX4 (`mc_att_control + mc_rate_control`) |
| `1` | Robust quaternion controller (this work) — **default** |

To switch from QGroundControl: **Vehicle Configuration → Parameters → search `MC_RATT_USE`**.

Or from the PX4 console:

```bash
# Use robust controller (default)
param set MC_RATT_USE 1

# Use default PX4 controller
param set MC_RATT_USE 0
```

---

## Flight Procedure

### Basic hover test

1. Launch simulation
2. In QGC: arm and takeoff in **Position mode**
3. Climb to at least 3 meters
4. Switch to **Altitude mode**
5. Verify controller is running from PX4 console:

```bash
mc_robust_att_control status   # should say: running
mc_att_control status          # should say: not running
```

---

## Attitude Tracking Experiment (Paper Section V)

The controller includes an internal sinusoidal attitude trajectory for validation, matching the simulation in the paper.

### Activate trajectory

```bash
# While hovering stably in Altitude mode:
param set MC_RATT_TRAJ 1
```

The drone will track:
```
qd(t) = ZYX(ψ₀, py(t), px(t))

px(t) = A_roll  · window(t) · sin(0.8t)   [roll  ±17°]
py(t) = A_pitch · window(t) · sin(1.2t)   [pitch ±17°]
ψ₀   = yaw at activation time (fixed)
```

where `window(t)` is a Hanning envelope for smooth start.

### Deactivate trajectory

```bash
param set MC_RATT_TRAJ 0
```

### Apply external disturbance (wrench injection)

In a separate terminal while the drone is flying:

```bash
# Apply torque disturbance
gz topic -p /gazebo/default/iris/base_link/wrench \
  -m "force: {x: 0, y: 0, z: 0} torque: {x: 5, y: 0, z: 0}"

# Remove disturbance
gz topic -p /gazebo/default/iris/base_link/wrench \
  -m "force: {x: 0, y: 0, z: 0} torque: {x: 0, y: 0, z: 0}"
```

---

## Controller Parameters

All parameters are in the `MC_RATT_*` group in QGroundControl (**Parameters → Multicopter Robust Attitude Control**).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MC_RATT_USE` | 1 | 0 = PX4 default, 1 = robust controller |
| `MC_RATT_K1` | 5.0 | Nonlinear coupling gain |
| `MC_RATT_K3` | 15.0 | Attitude restoring gain (stiffness) |
| `MC_RATT_K4` | 5.0 | Linear velocity error damping |
| `MC_RATT_KAP` | 0.8 | Nonlinear damping gain κ |
| `MC_RATT_JXX` | 0.029125 | Nominal inertia Jxx [kg·m²] |
| `MC_RATT_JYY` | 0.029125 | Nominal inertia Jyy [kg·m²] |
| `MC_RATT_JZZ` | 0.055225 | Nominal inertia Jzz [kg·m²] |
| `MC_RATT_TMAX_X` | 0.50 | Max roll torque [N·m] |
| `MC_RATT_TMAX_Y` | 0.50 | Max pitch torque [N·m] |
| `MC_RATT_TMAX_Z` | 0.40 | Max yaw torque [N·m] |
| `MC_RATT_ODCUT` | 10.0 | Ωd_dot LPF cutoff [Hz] |
| `MC_RATT_TRAJ` | 0 | 0 = PX4 setpoint, 1 = internal trajectory |
| `MC_RATT_TDUR` | 5.0 | Trajectory duration [s] |

---

## Log Analysis

Logs are saved to:

```
build/px4_sitl_default/rootfs/log/<date>/
```

Find the most recent log:

```bash
find ~/PX4-QuatAttControl/build/px4_sitl_default/rootfs/log/ -name "*.ulg" | sort | tail -1
```

Upload to **https://review.px4.io** for visualization of roll, pitch, yaw tracking vs setpoint.

---

## Modified Files

The following PX4 files were modified to integrate the controller:

| File | Change |
|------|--------|
| `ROMFS/px4fmu_common/init.d/rc.mc_apps` | Controller selection via `MC_RATT_USE` |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp` | Preserve attitude setpoint during trajectory |
| `src/modules/mc_pos_control/MulticopterPositionControl.hpp` | Added subscription and state variables |
| `boards/px4/sitl/default.px4board` | Added `mc_robust_att_control` to build |

New module added:

```
src/modules/mc_robust_att_control/
├── CMakeLists.txt
├── Kconfig
├── mc_robust_att_control.hpp
├── mc_robust_att_control_main.cpp
└── mc_robust_att_control_params.c
```

---

## Citation

If you use this work, please cite:

```bibtex
@article{flores2026quaternion,
  title   = {Almost Global Exponential Attitude Stabilization via 
             Continuous Nonlinear Quaternion Feedback},
  author  = {Flores, Gerardo and Torres, Jorge},
  journal = {IEEE Transactions on Automatic Control},
  year    = {2026},
  note    = {Under review}
}
```

---

## Contact

**Gerardo Flores, Ph.D.**  
Associate Professor, RAPTOR Lab  
Texas A&M International University (TAMIU)  
gerardo.flores@tamiu.edu
