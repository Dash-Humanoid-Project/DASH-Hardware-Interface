#pragma once

// ===== ODrive CAN node IDs =====
// Each ODrive has a globally unique node ID, even across separate Teensys.
// Teensy 1 (left leg):  node IDs 0–4
// Teensy 2 (right leg): node IDs 5–9  (9 = r_ankle, reserved)
// Teensy 3 (left arm):  node IDs 5–8 — REUSES the right leg's node ID
//   values. This is intentional, not a collision: node_id only needs to be
//   unique per physical CAN bus, and Teensy 3 has its own isolated CAN1/
//   CAN2 wiring, electrically separate from Teensy 2's. Confirmed directly
//   by the user (2026-07-28) that the arm's ODrives were flashed with
//   node_id 5-8 to match their own physical board labeling. The macro
//   *names* below (ODRV10-13) are this project's own sequential indexing
//   scheme for array/Param.h bookkeeping and are unrelated to the node_id
//   value itself — don't confuse "ODRV10" the macro with node_id 10.
// Teensy 4 (right arm): node IDs 14–17 — genuinely unique this time (no
//   reuse needed), confirmed directly by the user (2026-07-28). Macro names
//   ODRV14-17 match the real node_id values here, unlike Teensy 3's offset
//   scheme above.
#define ODRV0_CAN_NODE_ID 0   // l_hip_yaw
#define ODRV1_CAN_NODE_ID 1   // l_hip_roll
#define ODRV2_CAN_NODE_ID 2   // l_hip_pitch
#define ODRV3_CAN_NODE_ID 3   // l_knee
#define ODRV4_CAN_NODE_ID 4   // l_ankle
#define ODRV5_CAN_NODE_ID 5   // r_hip_yaw
#define ODRV6_CAN_NODE_ID 6   // r_hip_roll
#define ODRV7_CAN_NODE_ID 7   // r_hip_pitch
#define ODRV8_CAN_NODE_ID 8   // r_knee
#define ODRV9_CAN_NODE_ID 9   // r_ankle (reserved)
#define ODRV10_CAN_NODE_ID 5  // l_shoulder_pitch (Teensy 3, own isolated CAN bus)
#define ODRV11_CAN_NODE_ID 6  // l_shoulder_roll  (Teensy 3, own isolated CAN bus)
#define ODRV12_CAN_NODE_ID 7  // l_shoulder_yaw   (Teensy 3, own isolated CAN bus)
#define ODRV13_CAN_NODE_ID 8  // l_elbow          (Teensy 3, own isolated CAN bus)
#define ODRV14_CAN_NODE_ID 14 // r_shoulder_pitch (Teensy 4, own isolated CAN bus)
#define ODRV15_CAN_NODE_ID 15 // r_shoulder_roll  (Teensy 4, own isolated CAN bus)
#define ODRV16_CAN_NODE_ID 16 // r_shoulder_yaw   (Teensy 4, own isolated CAN bus)
#define ODRV17_CAN_NODE_ID 17 // r_elbow          (Teensy 4, own isolated CAN bus)

// ===== Motors per CAN bus =====
// Teensy 1
#define N_ODRIVE_CAN1 2   // left leg: CAN1 carries l_hip_yaw + l_hip_roll
#define N_ODRIVE_CAN2 2   // left leg: CAN2 carries l_hip_pitch + l_knee
#define N_ODRIVE_CAN5 1   // left leg: l_ankle (1 real motor). This is its own
                          // logical SystemData bus (still padded to 2 slots,
                          // node 1 unused/always zero, for PC wire-protocol
                          // compatibility — see UPXtreme.cpp's hardcoded
                          // SystemData<2> per bus) even though it now
                          // physically shares the CAN2 wire with l_hip_pitch/
                          // l_knee rather than a dedicated CAN3 bus; CAN3 is
                          // currently unused on this Teensy.
// Teensy 2
#define N_ODRIVE_CAN3 2   // right leg: T2-CAN1 carries r_hip_yaw + r_hip_roll
#define N_ODRIVE_CAN4 2   // right leg: T2-CAN2 carries r_hip_pitch + r_knee
// Teensy 3
#define N_ODRIVE_CAN6 2   // left arm: T3-CAN1 carries l_shoulder_pitch + l_shoulder_roll
#define N_ODRIVE_CAN7 2   // left arm: T3-CAN2 carries l_shoulder_yaw + l_elbow
// Teensy 4
#define N_ODRIVE_CAN8 2   // right arm: T4-CAN1 carries r_shoulder_pitch + r_shoulder_roll
#define N_ODRIVE_CAN9 2   // right arm: T4-CAN2 carries r_shoulder_yaw + r_elbow

// ===== Order IDs: slot position within each CAN bus (0-indexed) =====
// Teensy 1
#define ODRV0_CAN_ORDER_ID 0
#define ODRV1_CAN_ORDER_ID 1
#define ODRV2_CAN_ORDER_ID 0
#define ODRV3_CAN_ORDER_ID 1
#define ODRV4_CAN_ORDER_ID 0
// Teensy 2
#define ODRV5_CAN_ORDER_ID 0
#define ODRV6_CAN_ORDER_ID 1
#define ODRV7_CAN_ORDER_ID 0
#define ODRV8_CAN_ORDER_ID 1
// Teensy 3
#define ODRV10_CAN_ORDER_ID 0
#define ODRV11_CAN_ORDER_ID 1
#define ODRV12_CAN_ORDER_ID 0
#define ODRV13_CAN_ORDER_ID 1
// Teensy 4
#define ODRV14_CAN_ORDER_ID 0
#define ODRV15_CAN_ORDER_ID 1
#define ODRV16_CAN_ORDER_ID 0
#define ODRV17_CAN_ORDER_ID 1

// ===== Bus IDs: which CAN bus within this Teensy (0-indexed) =====
// Teensy 1
#define ODRV0_CAN_BUS_ID 0
#define ODRV1_CAN_BUS_ID 0
#define ODRV2_CAN_BUS_ID 1
#define ODRV3_CAN_BUS_ID 1
#define ODRV4_CAN_BUS_ID 2  // logical SystemData bus only — unchanged so the
                            // PC-Teensy wire protocol doesn't need updating;
                            // physically back on its own CAN3 wire (see ODRV4_CAN below)
// Teensy 2
#define ODRV5_CAN_BUS_ID 0
#define ODRV6_CAN_BUS_ID 0
#define ODRV7_CAN_BUS_ID 1
#define ODRV8_CAN_BUS_ID 1
// Teensy 3
#define ODRV10_CAN_BUS_ID 0
#define ODRV11_CAN_BUS_ID 0
#define ODRV12_CAN_BUS_ID 1
#define ODRV13_CAN_BUS_ID 1
// Teensy 4
#define ODRV14_CAN_BUS_ID 0
#define ODRV15_CAN_BUS_ID 0
#define ODRV16_CAN_BUS_ID 1
#define ODRV17_CAN_BUS_ID 1

// ===== CAN peripheral aliases (Teensy 1) =====
#define ODRV0_CAN can1
#define ODRV1_CAN can1
#define ODRV2_CAN can2
#define ODRV3_CAN can2
#define ODRV4_CAN can3  // l_ankle: back on its own dedicated CAN3 bus (previously
                        // shared CAN2 with l_hip_pitch/l_knee; moved back after
                        // CAN2 bus load with 3 nodes caused audible jerking on
                        // l_hip_pitch/l_knee during --position testing)
// Teensy 2 uses the same physical peripherals (can1/can2) on its own board
#define ODRV5_CAN can1
#define ODRV6_CAN can1
#define ODRV7_CAN can2
#define ODRV8_CAN can2
// Teensy 3 uses the same physical peripherals (can1/can2) on its own board.
// 2+2 split (not all 4 on one bus) deliberately, per the CAN2 bandwidth
// lesson above — 3 nodes sharing one bus already caused audible jerking.
#define ODRV10_CAN can1
#define ODRV11_CAN can1
#define ODRV12_CAN can2
#define ODRV13_CAN can2
// Teensy 4 uses the same physical peripherals (can1/can2) on its own board.
// Same 2+2 split reasoning as Teensy 3.
#define ODRV14_CAN can1
#define ODRV15_CAN can1
#define ODRV16_CAN can2
#define ODRV17_CAN can2

// ===== Per-joint safety limits (wire units: turns, turns/s, Nm motor-shaft) =====
// Mirrors the PC-side MotorConfig limits in include/MotorConfig.h /
// HardwareBridge.cpp (joint-space rad/rad/s/Nm), converted through each
// joint's gear ratio (turns_per_rad = gear_ratio / 2π). These are enforced
// independently of the PC-side clamp in Leg.cpp as the firmware-level
// backstop — see teensy.ino's command handlers.
//
// PLACEHOLDERS: derived from motor motion already exercised in
// ClosedLoopControlTest.cpp's --position sweep (hip_yaw/roll/pitch, knee)
// and the joint-space torque bound already used in --cartesian mode.
// l_ankle/r_ankle were never swept — arbitrary conservative guesses.
// Must be re-verified before the full range of motion is used for
// standing/walking. Right-leg (ODRV5-8) values assume the same joint types
// and gear ratios as the mirrored left-leg joints.

// Teensy 1 (left leg) — hip_yaw/roll/pitch, knee: 10:1 gearbox; ankle: 36:1
#define ODRV0_Q_MIN_TURNS   -1.0f
#define ODRV0_Q_MAX_TURNS    1.0f
#define ODRV0_TAU_MAX_NM     0.2f
#define ODRV0_VEL_MAX_TURNS_S 1.591549f

#define ODRV1_Q_MIN_TURNS   -1.0f
#define ODRV1_Q_MAX_TURNS    1.0f
#define ODRV1_TAU_MAX_NM     0.2f
#define ODRV1_VEL_MAX_TURNS_S 1.591549f

#define ODRV2_Q_MIN_TURNS   -3.0f
#define ODRV2_Q_MAX_TURNS    0.0f
#define ODRV2_TAU_MAX_NM     0.2f
#define ODRV2_VEL_MAX_TURNS_S 2.387324f

#define ODRV3_Q_MIN_TURNS   -3.0f
#define ODRV3_Q_MAX_TURNS    0.0f
#define ODRV3_TAU_MAX_NM     0.2f
#define ODRV3_VEL_MAX_TURNS_S 2.387324f

#define ODRV4_Q_MIN_TURNS   -1.719f
#define ODRV4_Q_MAX_TURNS    1.719f
#define ODRV4_TAU_MAX_NM     0.027778f
#define ODRV4_VEL_MAX_TURNS_S 2.864789f

// Teensy 2 (right leg) — hip_yaw/roll/pitch, knee: 10:1 gearbox (r_ankle/ODRV9 unused)
#define ODRV5_Q_MIN_TURNS   -1.0f
#define ODRV5_Q_MAX_TURNS    1.0f
#define ODRV5_TAU_MAX_NM     0.2f
#define ODRV5_VEL_MAX_TURNS_S 1.591549f

#define ODRV6_Q_MIN_TURNS   -1.0f
#define ODRV6_Q_MAX_TURNS    1.0f
#define ODRV6_TAU_MAX_NM     0.2f
#define ODRV6_VEL_MAX_TURNS_S 1.591549f

#define ODRV7_Q_MIN_TURNS   -3.0f
#define ODRV7_Q_MAX_TURNS    0.0f
#define ODRV7_TAU_MAX_NM     0.2f
#define ODRV7_VEL_MAX_TURNS_S 2.387324f

#define ODRV8_Q_MIN_TURNS   -3.0f
#define ODRV8_Q_MAX_TURNS    0.0f
#define ODRV8_TAU_MAX_NM     0.2f
#define ODRV8_VEL_MAX_TURNS_S 2.387324f

// Teensy 3 (left arm) — shoulder_pitch/roll/yaw, elbow: 10:1 gearbox
// (confirmed by user 2026-07-28).
//
// shoulder_roll (ODRV11) / shoulder_yaw (ODRV12): position stays at the
// unchanged placeholder, mirrored from ODRV0/ODRV1 (l_hip_yaw/l_hip_roll) —
// a real hand-guided recording (2026-07-28) stayed well inside +/-1.0 turns
// on both, no evidence yet it needs widening.
//
// shoulder_pitch (ODRV10) / elbow (ODRV13): an early recording repeatedly
// hit these position limits during ordinary careful guided motion
// (shoulder_pitch to 1.99 rad, elbow to -0.90 rad) — a hip_yaw/roll-sized
// limit was simply the wrong reference for a shoulder/elbow's naturally
// larger range of motion. shoulder_pitch was widened to that observed range
// plus margin. elbow was too, but a LATER recording then reached +0.41 rad,
// clamping again — elbow's observed range across both recordings is
// [-0.90, 0.41] rad, so this time it's widened more generously to
// [-1.0, 0.6] rad rather than re-chasing the exact edge again. Still a
// placeholder, not a verified true mechanical limit. Matches
// HardwareBridge.cpp's PC-side clamp: turns = rad * turns_per_rad,
// turns_per_rad = 10/(2*pi) = 1.591549.
//
// VEL_MAX_TURNS_S raised to 4.774648 (3.0 rad/s) on all four — was
// 1.591549 (1.0 rad/s), the hip-mirrored placeholder. The same recording's
// finite-difference velocity repeatedly hit that on every joint, not just
// pitch/elbow (roll peaked ~2.47 rad/s, yaw ~2.14 rad/s) — same
// "wrong reference joint" reasoning, not independently re-derived per joint.
#define ODRV10_Q_MIN_TURNS   -0.318310f  // -0.2 rad
#define ODRV10_Q_MAX_TURNS    3.421830f  //  2.15 rad
#define ODRV10_TAU_MAX_NM     0.2f
#define ODRV10_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

#define ODRV11_Q_MIN_TURNS   -1.0f
#define ODRV11_Q_MAX_TURNS    1.0f
#define ODRV11_TAU_MAX_NM     0.2f
#define ODRV11_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

// Q_MIN widened 2026-07-30: a --both bimanual recording pushed
// l_shoulder_yaw to -1.21 rad, clamping against the never-before-hit
// +/-0.63 rad placeholder. Positive side left unchanged (never approached).
#define ODRV12_Q_MIN_TURNS   -2.069014f  // -1.3 rad
#define ODRV12_Q_MAX_TURNS    1.0f       //  0.63 rad
#define ODRV12_TAU_MAX_NM     0.2f
#define ODRV12_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

#define ODRV13_Q_MIN_TURNS   -1.591549f  // -1.0 rad
#define ODRV13_Q_MAX_TURNS    0.954930f  //  0.6 rad
#define ODRV13_TAU_MAX_NM     0.2f
#define ODRV13_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

// Teensy 4 (right arm) — shoulder_pitch/roll/yaw, elbow: 10:1 gearbox
// (confirmed by user 2026-07-28, same as left arm). Seeded directly from
// the left arm's own current, battle-tested values (ODRV10-13 above,
// widened multiple times from real hand-guided recordings — see
// [[dash-arm-trajectory-playback]] in project memory) rather than
// restarting from the naive hip_yaw/roll-mirrored placeholder — same
// physical arm design mirrored, so this is a much better-informed starting
// point. Still a placeholder: not independently verified against the right
// arm's own real range of motion. Expect this to need adjusting once the
// right arm is actually exercised, same as the left arm was.
// Q_MIN widened 2026-07-30: the same --both bimanual recording pushed
// r_shoulder_pitch to -0.97 rad — well past the seeded -0.2 min, since the
// left arm's own recording never went that negative. Max side unchanged.
#define ODRV14_Q_MIN_TURNS   -1.750704f  // -1.1 rad
#define ODRV14_Q_MAX_TURNS    3.421830f  //  2.15 rad
#define ODRV14_TAU_MAX_NM     0.2f
#define ODRV14_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

#define ODRV15_Q_MIN_TURNS   -1.0f       // -0.63 rad, matches ODRV11 (shoulder_roll)
#define ODRV15_Q_MAX_TURNS    1.0f       //  0.63 rad
#define ODRV15_TAU_MAX_NM     0.2f
#define ODRV15_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

// Q_MAX widened 2026-07-30: same recording pushed r_shoulder_yaw to
// 1.02 rad, past the seeded +0.63 max. Min side unchanged.
#define ODRV16_Q_MIN_TURNS   -1.0f       // -0.63 rad, matches ODRV12 (shoulder_yaw)
#define ODRV16_Q_MAX_TURNS    1.830282f  //  1.15 rad
#define ODRV16_TAU_MAX_NM     0.2f
#define ODRV16_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

#define ODRV17_Q_MIN_TURNS   -1.591549f  // -1.0 rad, matches ODRV13 (elbow)
#define ODRV17_Q_MAX_TURNS    0.954930f  //  0.6 rad
#define ODRV17_TAU_MAX_NM     0.2f
#define ODRV17_VEL_MAX_TURNS_S 4.774648f // 3.0 rad/s

// ===== Comms-loss watchdog =====
// If no CRC-valid command is received within this window, the Teensy idles
// all ODrives on its own rather than continuing to execute a stale command
// indefinitely (see teensy.ino's watchdog_tripped check in loop()).
// Placeholder: ~75x the nominal 2ms command interval — long enough to
// absorb normal jitter, short enough to catch a real stall quickly. Adjust
// here if it trips too eagerly or too late in practice.
#define WATCHDOG_TIMEOUT_MS 150

// ===== Network config =====
#define TEENSY1_IP "10.176.32.33"
#define TEENSY2_IP "10.176.32.34"
#define TEENSY3_IP "10.176.32.35"  // ASSUMED — not yet confirmed against the
                                   // physical Teensy 3's flashed static IP.
                                   // Verify/set to match before first boot.
#define TEENSY4_IP "10.176.32.36"  // ASSUMED — same caveat as TEENSY3_IP,
                                   // following the same sequential pattern.
                                   // Verify/set to match before first boot.

// ===== Command buffer sizing =====
// Maximum serialized payload for any command (excluding type byte and CRC).
// Worst case: PositionCommand or SetGainsCommand with 8 motors, all fields
// now full floats (Vel_FF/Torque_FF were int16 until the LegController work —
// see include/Command.h — which truncated them to whole-unit precision)
//   = 1 (motor count) + 8*4 (pos/pos_gain) + 8*4 (vel_ff/vel_gain)
//     + 8*4 (torque_ff/vel_integrator_gain) = 97 bytes
// 128 bytes gives comfortable headroom.
#define MAX_CMD_PAYLOAD_SIZE 128
