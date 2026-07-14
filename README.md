# DASH Hardware Interface

C++ control library for DASH's legs. A PC-side control application talks to Teensy 4.1 microcontrollers over UDP; each Teensy bridges to that leg's ODrive Pro motor controllers over CAN.

```
PC (control logic, C++17 / ASIO)
  |  UDP, ~500 Hz command/feedback
Teensy 4.1  (CRC-checked relay + per-joint safety clamp + comms-loss watchdog)
  |  CAN, 250 kbps
ODrive Pro  (per-joint closed-loop position/velocity/torque control, ~8 kHz internal loop)
```

One Teensy per leg. Currently only the left leg is wired up on the PC side; the right leg's firmware exists but isn't yet instantiated by `HardwareBridge`.

## Repository layout

- **`include/`** — shared headers. This is the single source of truth: CMake copies every header in this directory verbatim into `teensy/`, `teensy2/`, and the two CAN benchmark test directories on every build. **Edit only here** — the copies are overwritten on the next build.
- **`src/`** — PC-side implementation (`.cpp` files for the classes below).
- **`teensy/`** — hand-maintained Arduino firmware for Teensy 1 (left leg), plus the auto-copied shared headers. `teensy.ino` is not auto-generated and is not built by CMake — it's flashed separately via the Arduino IDE.
- **`teensy2/`** — same, for Teensy 2 (right leg).
- **`test/`** — PC-side test/diagnostic executables (see [Running](#running) below) and two Teensy-only CAN benchmark sketches.
- **`logs/`** — CSV output from `--position` runs, plus Python scripts for analyzing loop timing (`check_timestep.py`, `check_timestep_detailed.py`) and plotting encoder data (`encoder_reading_plotter.py`, `frequency_plotter.py`).
- **`Dash_URDF/`** — the robot's URDF, including real per-link mass/inertia data (not placeholders) — not yet consumed by any C++ dynamics code, but available for future whole-body work.

## Core classes

- **`HardwareBridge`** — top-level entry point. Owns the `UPXtreme` connections (one per Teensy) and exposes each leg by name. Construct one, call `start()`, then interact only through `leftLeg()`/`rightLeg()`.
- **`Leg`** — one Teensy's motors as a named group (`MotorConfig` list + a `UPXtreme&`). All public methods (`getJointStates()`, `setPositions()`, `setVelocities()`, `setTorques()`, `setGains()`) use SI units (rad, rad/s, Nm) — internal conversion to ODrive turns accounts for each joint's real gear ratio (see [Safety systems](#safety-systems)). Every commanded value is clamped against that joint's configured limits before being sent, with a `std::cerr` warning if a clamp actually triggers.
- **`LegController`** — unified joint-space + Cartesian-space PD/impedance primitive (`include/LegController.h`), modeled on Cheetah-Software's `LegController`. Joint-space targets (`qDes`/`qdDes`) and gains (`kpJoint`/`kdJoint`, dispatched via `SetGainsCommand`) are tracked *locally* by the ODrive's own onboard position/velocity loop — this class never sends a raw open-loop torque override for joint tracking. Only the Cartesian-space term (`kpCartesian`/`kdCartesian`/`pDes`/`vDes`/`forceFeedForward`, scoped to the `l_hip_yaw`→`l_hip_roll`→`l_hip_pitch`→`l_knee` kinematic chain) is computed on the PC each cycle and folded in as additive feedforward torque via `J^T`. Also exposes a `tauEstimate()` diagnostic (computed, not measured).
- **`MotorConfig`** — static per-joint config: `joint_name`, `bus_idx`/`node_idx` (CAN routing), `turns_per_rad` (= `gear_ratio / 2π`), and the safety limits `q_min_rad`/`q_max_rad`/`tau_max_nm`/`vel_max_rad_s`.
- **`JointState`** — measured `position_rad`/`velocity_rad_s` for one joint, after gear-ratio conversion.
- **`UPXtreme`** — the UDP transport to one Teensy: send/receive threads, command serialization, thread-safe feedback accessors. **`SimUPXtreme`** is a drop-in software substitute (first-order-lag position/velocity/torque dynamics, no real network/CAN) for `--sim` runs — useful for compile and wiring sanity checks, not for validating timing or ODrive-specific behavior.
- **`LeftLegKinematics.h`** (`LeftLeg::` namespace) — forward kinematics and a numerical Jacobian for the 4-joint chain, derived from the URDF. `l_ankle` is outside this chain (the URDF places the foot at zero offset beyond it, so it has no effect on end-effector position with the current model).
- **`Imu`** — thin wrapper around a VectorNav IMU's `EzAsyncData` API (USB-serial). Provides roll/pitch/yaw and body-frame angular rate; not yet fused with anything (no body-frame state estimator exists yet).
- **`PeriodicTimer`** (`include/PeriodicTimer.h`) — `timerfd`-anchored fixed-rate scheduler for the PC-side control loop. `wait()` blocks until the next tick on a schedule fixed at construction, so a slow iteration is reported as a miss (`lastMissedTicks()`) rather than permanently shifting the phase of future ticks the way `std::this_thread::sleep_for` would.

## Wire protocol

Defined in `MsgBase.h`/`Command.h`. Every message is `[type byte][payload][CRC-8]`.

| `MsgType` | Purpose |
|---|---|
| `PositionCommand` | Per-motor `Input_Pos` (turns), `Vel_FF` (turns/s), `Torque_FF` (motor-shaft Nm) — the last two are additive feedforward on top of the ODrive's own local PD. |
| `VelocityCommand` | Per-motor `Input_Vel` + a shared `Input_Torque_FF`. |
| `TorqueCommand` | Per-motor raw `Input_Torque` (motor-shaft Nm) — open-loop, no local tracking. |
| `SetGains` | Per-motor `Pos_Gain`/`Vel_Gain`/`Vel_Integrator_Gain` — sets the ODrive's own onboard gains directly. |
| `IdleCommand` / `StartCommand` | Put all ODrives on this Teensy into `IDLE` / `CLOSED_LOOP_CONTROL`. |
| `Heartbeat` | No-op keep-alive — resets the comms-loss watchdog timer only, no `setState()`/`setControllerMode()`/motor command of any kind. Exists specifically so a settling wait doesn't need to send a real command (which would have side effects — see below) just to avoid tripping the watchdog. |
| `SystemData` | Teensy → PC feedback: per-bus encoder position/velocity estimates. |

Feedback comes back as one `SystemData<N>` block per CAN bus, aggregated in a `SystemDataContainer`.

## Safety systems

- **Gear ratios matter and are joint-specific.** `l_hip_yaw`/`l_hip_roll`/`l_hip_pitch`/`l_knee` are 10:1 planetary-geared; `l_ankle` is 36:1. `turns_per_rad = gear_ratio / (2π)` is computed per joint in `HardwareBridge.cpp`, not treated as uniform/direct-drive. Torque commands are divided by the joint's gear ratio before being sent as motor-shaft Nm (a gearbox multiplies torque by the ratio, so the motor only needs to supply `τ_joint / N`).
- **Per-joint clamp, enforced twice, independently.** `Leg`'s setters clamp on the PC before anything is sent (and log when they do); `teensy.ino`/`teensy2.ino` clamp again on the firmware side as a hardware-level backstop that works even if the PC sends something wrong. Limits live in `MotorConfig` (PC, SI units) and mirrored `#define`s in `Param.h` (Teensy, wire units) — **current limits are placeholders** derived from motion already exercised in testing, not validated hardware ratings; `l_ankle`'s in particular were never swept and are an arbitrary conservative guess.
- **Comms-loss watchdog.** If no CRC-valid command reaches a Teensy within `WATCHDOG_TIMEOUT_MS` (`Param.h`, currently 150 ms), it idles every ODrive on that bus. Recovery requires an explicit `StartCommand` — there's no silent auto-resume, matching the intended manual recovery workflow (power off, physical reset). Because of this, every drive-mode in `ClosedLoopControlTest.cpp` self-arms at startup rather than depending on a separate prior `--start` run, and any settling wait uses the `Heartbeat` message rather than a real command (sending a real `TorqueCommand`/`PositionCommand` as a keep-alive would force an unwanted ODrive controller-mode switch).

## Build

Requires a checkout of vnproglib (VectorNav SDK, used by `Imu`) — point `-DVNPROGLIB_DIR` at its `cpp/` directory if it's not at the CMake default (`/home/dvolpi/Source/vnproglib/cpp`).

```
mkdir build && cd build
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON [-DVNPROGLIB_DIR=/path/to/vnproglib/cpp]
make -j$(nproc)
```

Produces `closed_loop_test`, `test_udp_benchmark`, and `imu_test`. Zero build errors expected; two pre-existing, harmless `UPXTREME_i14 redefined` warnings from files that both define it (command-line and file-level).

Teensy firmware (`teensy/teensy.ino`, `teensy2/teensy2.ino`) is **not** part of this build — flash it separately via the Arduino IDE after the header-copy step above has run at least once.

## Hardware setup

Closed-loop control follows [Controlling ODrive from an Arduino via CAN](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html).

1. Static IPs (`include/Param.h`): Teensy 1 (left leg) `10.176.32.33:8000`, Teensy 2 (right leg) `10.176.32.34:8001`. Set the PC's CAN-facing interface name in `SystemConfig.h` (`PC_network_interface_name`).
2. Configure each ODrive per the guide's [ODrive configuration steps](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html#configuring-the-odrive) — node IDs must match `Param.h`'s `ODRVn_CAN_NODE_ID` assignments (0-4 for the left leg's `l_hip_yaw`/`l_hip_roll`/`l_hip_pitch`/`l_knee`/`l_ankle`, 5-8 reserved for the right leg).
3. Wire the CAN bus with a common ground per the [CAN hardware guide](https://docs.odriverobotics.com/v/latest/guides/can-guide.html#hardware-setup). On Teensy 1, `l_ankle` shares the CAN2 physical bus with `l_hip_pitch`/`l_knee` (CAN3 hardware initializes but is currently unused).
4. `motor.torque_constant` should be configured with the motor's true Kt (not a pre-gear-corrected value) — the gear-ratio division described above assumes this.

## Running

```
cd build
./closed_loop_test [--position | --velocity | --torque | --impedance | --cartesian | --start | --idle | --reset] [--sim]
```

| Flag | Behavior |
|---|---|
| `--position` | Sine-sweep position tracking on all 5 left-leg joints, via `Leg::setPositions()` directly (ODrive's own local position/velocity gains). Logs to `logs/position_measurement_log.csv`, including a `missed_ticks` column from `PeriodicTimer`. |
| `--velocity` | Holds zero velocity — exercises ODrive's `VELOCITY_CONTROL` axis mode directly. |
| `--torque` | Holds zero torque — exercises ODrive's `TORQUE_CONTROL` axis mode directly, open-loop. |
| `--impedance` | Joint-space virtual spring-damper around the leg's position at start, via `LegController`. |
| `--cartesian` | Cartesian-space virtual spring-damper on end-effector position (4-joint chain), via `LegController`. |
| `--start` / `--idle` | One-shot diagnostic arm/idle. `--start`'s effect reverts via the watchdog shortly after the process exits, since nothing keeps streaming — expected, not a bug. |
| `--reset` | Self-arms, then smoothly ramps every joint back to zero over 2 seconds. |
| `--sim` | Add to any of the above to run against `SimUPXtreme` instead of real hardware. |

Ctrl+C during any drive mode triggers a safe shutdown (zero torque/gains as appropriate for that mode).

Other executables: `test_udp_benchmark` (PC↔Teensy round-trip latency/jitter/packet-loss measurement — see the doc comment in `test/udp_benchmark/test_udp_benchmark.cpp`), `imu_test` (prints live IMU roll/pitch/yaw/gyro from a given serial port, default `/dev/ttyUSB0`). `test/CAN_cyclic_msg_benchmark` and `test/CAN_noncyclic_msg_benchmark` are standalone Teensy sketches (flashed independently, not part of the CMake build) for characterizing ODrive CAN message timing/loss directly, without the UDP layer.

## Status

Actively under development toward standing and eventually walking, working up from validated single-leg tabletop control. Safety hardening (per-joint clamps, comms-loss watchdog, fixed-rate control loop) and the `LegController` compliant-control primitive are complete; multi-limb generalization, whole-body balance, and gait are not yet started.
