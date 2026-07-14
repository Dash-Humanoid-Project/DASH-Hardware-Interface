#pragma once

// ===== ODrive CAN node IDs =====
// Each ODrive has a globally unique node ID, even across separate Teensys.
// Teensy 1 (left leg):  node IDs 0–4
// Teensy 2 (right leg): node IDs 5–9  (9 = r_ankle, reserved)
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

// ===== Bus IDs: which CAN bus within this Teensy (0-indexed) =====
// Teensy 1
#define ODRV0_CAN_BUS_ID 0
#define ODRV1_CAN_BUS_ID 0
#define ODRV2_CAN_BUS_ID 1
#define ODRV3_CAN_BUS_ID 1
#define ODRV4_CAN_BUS_ID 2  // logical SystemData bus only — physically shares
                            // the CAN2 wire with ODRV2/ODRV3 (see ODRV4_CAN below)
// Teensy 2
#define ODRV5_CAN_BUS_ID 0
#define ODRV6_CAN_BUS_ID 0
#define ODRV7_CAN_BUS_ID 1
#define ODRV8_CAN_BUS_ID 1

// ===== CAN peripheral aliases (Teensy 1) =====
#define ODRV0_CAN can1
#define ODRV1_CAN can1
#define ODRV2_CAN can2
#define ODRV3_CAN can2
#define ODRV4_CAN can2  // l_ankle: moved from CAN3 to CAN2 (shares the physical
                        // bus with l_hip_pitch/l_knee); CAN3 is currently unused
// Teensy 2 uses the same physical peripherals (can1/can2) on its own board
#define ODRV5_CAN can1
#define ODRV6_CAN can1
#define ODRV7_CAN can2
#define ODRV8_CAN can2

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

// ===== Command buffer sizing =====
// Maximum serialized payload for any command (excluding type byte and CRC).
// Worst case: PositionCommand or SetGainsCommand with 8 motors, all fields
// now full floats (Vel_FF/Torque_FF were int16 until the LegController work —
// see include/Command.h — which truncated them to whole-unit precision)
//   = 1 (motor count) + 8*4 (pos/pos_gain) + 8*4 (vel_ff/vel_gain)
//     + 8*4 (torque_ff/vel_integrator_gain) = 97 bytes
// 128 bytes gives comfortable headroom.
#define MAX_CMD_PAYLOAD_SIZE 128
