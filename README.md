# DASH Hardware Interface

C++ control library for DASH's four limbs (two legs, two arms). A PC-side control application talks to Teensy 4.1 microcontrollers over UDP; each Teensy bridges to that limb's ODrive Pro motor controllers over CAN.

```
PC (control logic, C++17 / ASIO)
  |  UDP, ~500 Hz command/feedback
Teensy 4.1  (CRC-checked relay + per-joint safety clamp + comms-loss watchdog)
  |  CAN, 250 kbps
ODrive Pro  (per-joint closed-loop position/velocity/torque control, ~8 kHz internal loop)
```

One Teensy per limb. All four (Teensy 1/2/3/4 = left leg/right leg/left arm/right arm) are wired up on the PC side and instantiated by `HardwareBridge`.

## Repository layout

- **`include/`** — shared headers. This is the single source of truth: CMake copies every header in this directory verbatim into `teensy/`, `teensy2/`, `teensy3/`, `teensy4/`, and the two CAN benchmark test directories on every build. **Edit only here** — the copies are overwritten on the next build.
- **`src/`** — PC-side implementation (`.cpp` files for the classes below).
- **`teensy/`** — hand-maintained Arduino firmware for Teensy 1 (left leg), plus the auto-copied shared headers. `teensy.ino` is not auto-generated and is not built by CMake — it's flashed separately via the Arduino IDE.
- **`teensy2/`** / **`teensy3/`** / **`teensy4/`** — same, for Teensy 2 (right leg), Teensy 3 (left arm), and Teensy 4 (right arm) respectively.
- **`test/`** — PC-side test/diagnostic executables (see [Running](#running) below) and two Teensy-only CAN benchmark sketches.
- **`logs/`** — CSV output from `--position` and `--record`/`--playback` runs, plus Python scripts for analyzing loop timing (`check_timestep.py`, `check_timestep_detailed.py`) and plotting encoder data (`encoder_reading_plotter.py`, `frequency_plotter.py`).
- **`Dash_URDF/`** — the robot's URDF, including real per-link mass/inertia data (not placeholders) — not yet consumed by any C++ dynamics code, but available for future whole-body work.

## Core classes

- **`HardwareBridge`** — top-level entry point. Owns the `UPXtreme` connections (one per Teensy) and exposes each limb by name. Construct one, call `start()`, then interact only through `leftLeg()`/`rightLeg()`/`leftArm()`/`rightArm()`.
- **`Leg`** — one Teensy's motors as a named group (`MotorConfig` list + a `UPXtreme&`) — despite the name, used for arms too, not leg-specific. All public methods (`getJointStates()`, `setPositions()`, `setVelocities()`, `setTorques()`, `setGains()`) use SI units (rad, rad/s, Nm) — internal conversion to ODrive turns accounts for each joint's real gear ratio (see [Safety systems](#safety-systems)). Every commanded value is clamped against that joint's configured limits before being sent, with a `std::cerr` warning if a clamp actually triggers.
- **`LegController`** — unified joint-space + Cartesian-space PD/impedance primitive (`include/LegController.h`), modeled on Cheetah-Software's `LegController`. Joint-space targets (`qDes`/`qdDes`) and gains (`kpJoint`/`kdJoint`, dispatched via `SetGainsCommand`) are tracked *locally* by the ODrive's own onboard position/velocity loop — this class never sends a raw open-loop torque override for joint tracking. Only the Cartesian-space term (`kpCartesian`/`kdCartesian`/`pDes`/`vDes`/`forceFeedForward`) is computed on the PC each cycle and folded in as additive feedforward torque via `J^T`. Genericized over limb type via the `LimbKinematics` injection seam (below) — one `LegController` instance per limb, each bound to that limb's own kinematics at construction. Also exposes a `tauEstimate()` diagnostic (computed, not measured).
- **`LimbKinematics`** (`include/LimbKinematics.h`) — a struct of function pointers (forward kinematics, Jacobian, `J^T` multiply, joint names), not an abstract base class — this codebase has no runtime polymorphism anywhere, and which kinematics a `LegController` uses is decided once at construction, never swapped at runtime. Each limb's real implementation is transcribed directly from `Dash_URDF/dash.urdf`: `LeftLegKinematics.h`/`RightLegKinematics.h`/`LeftArmKinematics.h`/`RightArmKinematics.h` (`LeftLeg::`/`RightLeg::`/`LeftArm::`/`RightArm::` namespaces). The right leg's chain is **not** a sign-flipped mirror of the left — the URDF uses a reflection+π rotation convention, so identical joint commands to both legs do not produce visually mirrored motion; that needs deliberate per-joint handling in any future gait work.
- **`MotorConfig`** — static per-joint config: `joint_name`, `bus_idx`/`node_idx` (CAN routing), `turns_per_rad` (= `gear_ratio / 2π`), and the safety limits `q_min_rad`/`q_max_rad`/`tau_max_nm`/`vel_max_rad_s`.
- **`JointState`** — measured `position_rad`/`velocity_rad_s` for one joint, after gear-ratio conversion.
- **`UPXtreme`** — the UDP transport to one Teensy: send/receive threads, command serialization, thread-safe feedback accessors. **`SimUPXtreme`** is a drop-in software substitute (first-order-lag position/velocity/torque dynamics, no real network/CAN) for `--sim` runs — useful for compile and wiring sanity checks, not for validating timing or ODrive-specific behavior.
- **`Mode`** (`include/Mode.h`) — base class (`onEnter`/`run`/`onExit`/`canExit`) for a live-switchable control mode, modeled on Cheetah-Software's `FSM_State<T>`. `PositionMode`/`ImpedanceMode`/`CartesianMode` (`test/*.cpp`) each implement it, driving all four limbs at once.
- **`ModeDispatcher`** (`include/ModeDispatcher.h`) — owns the current `Mode` and switches it on a mapped keypress via `KeyboardInput`. `KeyboardInput` (`include/KeyboardInput.h`) is a RAII termios raw/non-blocking stdin wrapper that degrades gracefully (no-op) if stdin isn't a TTY.
- **`TrajectoryPlayback`** (`test/TrajectoryPlayback.h`) — hand-guided record/playback for the arms: idles the target arm so it can be moved by hand, samples `getJointStates()` at 50 Hz, and can later ramp back in and stream the recording back with velocity feedforward from finite differences. Backs the `--record`/`--playback` CLI flags below.
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

- **Every ODrive has a globally unique CAN node ID**, even across separate Teensys: Teensy 1 (left leg) 0-4, Teensy 2 (right leg) 5-9 (9 = reserved `r_ankle`, not physically installed), Teensy 3 (left arm) 10-13, Teensy 4 (right arm) 14-17. `Param.h`'s `ODRVn_CAN_NODE_ID` macro names match these values exactly for every Teensy.
- **Gear ratios matter and are joint-specific.** Hip/knee/shoulder/elbow joints (both legs, both arms) are 10:1 planetary-geared; `l_ankle` is 36:1 (the right leg has no physical ankle yet). `turns_per_rad = gear_ratio / (2π)` is computed per joint in `HardwareBridge.cpp`, not treated as uniform/direct-drive. Torque commands are divided by the joint's gear ratio before being sent as motor-shaft Nm (a gearbox multiplies torque by the ratio, so the motor only needs to supply `τ_joint / N`).
- **Per-joint clamp, enforced twice, independently.** `Leg`'s setters clamp on the PC before anything is sent (and log when they do); each `teensyN.ino` clamps again on the firmware side as a hardware-level backstop that works even if the PC sends something wrong. Limits live in `MotorConfig` (PC, SI units) and mirrored `#define`s in `Param.h` (Teensy, wire units) — **current limits are placeholders** derived from motion already exercised in testing, not validated hardware ratings. Both arms' limits have been widened multiple times from real hand-guided recordings; the right leg's are still mirrored from the left leg's numbers, unvalidated against its own real range of motion.
- **Comms-loss watchdog.** If no CRC-valid command reaches a Teensy within `WATCHDOG_TIMEOUT_MS` (`Param.h`, currently 150 ms), it idles every ODrive on that bus. Recovery requires an explicit `StartCommand` — there's no silent auto-resume, matching the intended manual recovery workflow (power off, physical reset). Because of this, every drive-mode in `ClosedLoopControlTest.cpp` self-arms at startup rather than depending on a separate prior `--start` run, and any settling wait uses the `Heartbeat` message rather than a real command (sending a real `TorqueCommand`/`PositionCommand` as a keep-alive would force an unwanted ODrive controller-mode switch).

## Build

Requires a checkout of vnproglib (VectorNav SDK, used by `Imu`) — point `-DVNPROGLIB_DIR` at its `cpp/` directory if it's not at the CMake default (`/home/dvolpi/Source/vnproglib/cpp`).

```
mkdir build && cd build
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON [-DVNPROGLIB_DIR=/path/to/vnproglib/cpp]
make -j$(nproc)
```

Produces `closed_loop_test`, `test_udp_benchmark`, and `imu_test`. Zero build errors expected; two pre-existing, harmless `UPXTREME_i14 redefined` warnings from files that both define it (command-line and file-level).

Teensy firmware (`teensy/teensy.ino`, `teensy2/teensy2.ino`, `teensy3/teensy3.ino`, `teensy4/teensy4.ino`) is **not** part of this build — flash each one separately via the Arduino IDE, once per physical board, after the header-copy step above has run at least once. The shared headers are identical across all four folders; the `.ino` sketches are not (different IP/port/node IDs/CAN filter values per board) — flashing one board's sketch onto another Teensy is not meaningful.

## Hardware setup

Closed-loop control follows [Controlling ODrive from an Arduino via CAN](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html).

1. Static IPs (`include/Param.h`): Teensy 1 (left leg) `10.176.32.33:8000`, Teensy 2 (right leg) `10.176.32.34:8001`, Teensy 3 (left arm) `10.176.32.35:8002`, Teensy 4 (right arm) `10.176.32.36:8003`. Set the PC's CAN-facing interface name in `SystemConfig.h` (`PC_network_interface_name`).
2. Configure each ODrive per the guide's [ODrive configuration steps](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html#configuring-the-odrive) — node IDs must match `Param.h`'s `ODRVn_CAN_NODE_ID` assignments (Teensy 1: 0-4 for `l_hip_yaw`/`l_hip_roll`/`l_hip_pitch`/`l_knee`/`l_ankle`; Teensy 2: 5-8 for the equivalent right-leg joints, 9 reserved for a not-yet-installed `r_ankle`; Teensy 3: 10-13 for `l_shoulder_pitch`/`l_shoulder_roll`/`l_shoulder_yaw`/`l_elbow`; Teensy 4: 14-17 for the equivalent right-arm joints).
3. Wire the CAN bus with a common ground per the [CAN hardware guide](https://docs.odriverobotics.com/v/latest/guides/can-guide.html#hardware-setup). On Teensy 1, `l_ankle` shares the CAN2 physical bus with `l_hip_pitch`/`l_knee` (CAN3 hardware initializes but is currently unused). Each arm Teensy splits its 4 joints 2+2 across CAN1/CAN2 (not all 4 on one bus) — a 3-node single bus caused audible jerking during hip_pitch/knee testing, so the arms avoid repeating that.
4. `motor.torque_constant` should be configured with the motor's true Kt (not a pre-gear-corrected value) — the gear-ratio division described above assumes this.

## Running

```
cd build
./closed_loop_test [--position | --velocity | --torque | --impedance | --cartesian | --start | --idle | --reset | --record | --playback] [--sim] [--right | --both]
```

| Flag | Behavior |
|---|---|
| `--position` | Sine-sweep position tracking on all four limbs at once, via `Leg::setPositions()` directly (ODrive's own local position/velocity gains). Logs to `logs/position_measurement_log.csv`, including a `missed_ticks` column from `PeriodicTimer`. |
| `--impedance` | Joint-space virtual spring-damper around each limb's position at start, via `LegController`. |
| `--cartesian` | Cartesian-space virtual spring-damper on each limb's end-effector position, via `LegController`. |
| `--velocity` | Holds zero velocity on the left leg only — exercises ODrive's `VELOCITY_CONTROL` axis mode directly. Not yet generalized to all four limbs (structurally can't route through `LegController`). |
| `--torque` | Holds zero torque on the left leg only — exercises ODrive's `TORQUE_CONTROL` axis mode directly, open-loop. |
| `--start` / `--idle` | One-shot diagnostic arm/idle, all four limbs. `--start`'s effect reverts via the watchdog shortly after the process exits, since nothing keeps streaming — expected, not a bug. |
| `--reset` | Left leg only. Self-arms, then smoothly ramps every joint back to zero over 2 seconds. |
| `--record` / `--playback` | Hand-guided trajectory capture/replay for an arm: `--record` idles the target arm so it can be moved by hand and samples its motion; `--playback` streams it back. Left arm by default; add `--right` for the right arm, or `--both` to record/replay both arms simultaneously (own CSV file per case — `logs/arm_trajectory.csv`, `logs/arm_trajectory_right.csv`, `logs/both_arms_trajectory.csv`). |
| `--sim` | Add to any of the above to run against `SimUPXtreme` instead of real hardware. |

`--position`/`--impedance`/`--cartesian` all construct every `Mode` up front and share one live keyboard switcher — while one is running, press `p`/`i`/`c` to switch to another without restarting the process. Ctrl+C during any drive mode triggers a safe shutdown (zero torque/gains as appropriate for that mode, and a smooth ramp back toward home where applicable).

Other executables: `test_udp_benchmark` (PC↔Teensy round-trip latency/jitter/packet-loss measurement — see the doc comment in `test/udp_benchmark/test_udp_benchmark.cpp`), `imu_test` (prints live IMU roll/pitch/yaw/gyro from a given serial port, default `/dev/ttyUSB0`). `test/CAN_cyclic_msg_benchmark` and `test/CAN_noncyclic_msg_benchmark` are standalone Teensy sketches (flashed independently, not part of the CMake build) for characterizing ODrive CAN message timing/loss directly, without the UDP layer.

## Status

Actively under development toward standing and eventually walking, working up from validated single-leg tabletop control. Safety hardening (per-joint clamps, comms-loss watchdog, fixed-rate control loop), the `LegController` compliant-control primitive, and the `Mode`/`ModeDispatcher` live-switchable multi-limb architecture are complete — all four limbs are wired into `HardwareBridge` and drivable through `--position`/`--impedance`/`--cartesian` together. Both arms have been exercised on real hardware (including hand-guided record/playback); the right leg has not yet had a real-hardware run through this array-driven path, and its joint limits remain unvalidated placeholders mirrored from the left leg. Whole-body balance and gait are not yet started — see the architecture notes in-repo for the WBC/MPC gap analysis.
