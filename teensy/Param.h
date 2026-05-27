#pragma once

// ===== ODrive CAN node IDs =====
// Each ODrive has a globally unique node ID, even across separate Teensys.
// Teensy 1 (left leg):  node IDs 0–4  (4 = l_ankle, reserved)
// Teensy 2 (right leg): node IDs 5–9  (9 = r_ankle, reserved)
#define ODRV0_CAN_NODE_ID 0   // l_hip_yaw
#define ODRV1_CAN_NODE_ID 1   // l_hip_roll
#define ODRV2_CAN_NODE_ID 2   // l_hip_pitch
#define ODRV3_CAN_NODE_ID 3   // l_knee
#define ODRV4_CAN_NODE_ID 4   // l_ankle (reserved)
#define ODRV5_CAN_NODE_ID 5   // r_hip_yaw
#define ODRV6_CAN_NODE_ID 6   // r_hip_roll
#define ODRV7_CAN_NODE_ID 7   // r_hip_pitch
#define ODRV8_CAN_NODE_ID 8   // r_knee
#define ODRV9_CAN_NODE_ID 9   // r_ankle (reserved)

// ===== Motors per CAN bus =====
// Teensy 1
#define N_ODRIVE_CAN1 2   // left leg: CAN1 carries l_hip_yaw + l_hip_roll
#define N_ODRIVE_CAN2 2   // left leg: CAN2 carries l_hip_pitch + l_knee
// Teensy 2
#define N_ODRIVE_CAN3 2   // right leg: T2-CAN1 carries r_hip_yaw + r_hip_roll
#define N_ODRIVE_CAN4 2   // right leg: T2-CAN2 carries r_hip_pitch + r_knee

// ===== Order IDs: slot position within each CAN bus (0-indexed) =====
// Teensy 1
#define ODRV0_CAN_ORDER_ID 0
#define ODRV1_CAN_ORDER_ID 1
#define ODRV2_CAN_ORDER_ID 0
#define ODRV3_CAN_ORDER_ID 1
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
// Teensy 2 uses the same physical peripherals (can1/can2) on its own board
#define ODRV5_CAN can1
#define ODRV6_CAN can1
#define ODRV7_CAN can2
#define ODRV8_CAN can2

// ===== Network config =====
#define TEENSY1_IP "10.176.32.33"
#define TEENSY2_IP "10.176.32.34"

// ===== Command buffer sizing =====
// Maximum serialized payload for any command (excluding type byte and CRC).
// Worst case: PositionCommand with 8 motors
//   = 1 (motor count) + 8*4 (floats) + 8*2 (int16 vel_ff) + 2 (int16 torque_ff) = 51 bytes
// 64 bytes gives comfortable headroom.
#define MAX_CMD_PAYLOAD_SIZE 64
