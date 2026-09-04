// ===== Teensy 3 — Left arm firmware =====
// IP: 10.176.32.35 (ASSUMED — verify against the physical Teensy 3's
//     flashed static IP before first boot; see Param.h's TEENSY3_IP)
// Port: 8002
// ODrive node IDs: 10 (l_shoulder_pitch), 11 (l_shoulder_roll),
//                  12 (l_shoulder_yaw), 13 (l_elbow)
// CAN1: nodes 10, 11  |  CAN2: nodes 12, 13
//
// Relabeled 2026-09-02: node_id now matches the ODRV10-13 macro suffix
// exactly (previously node_id was 5-8, reused from the right leg's values —
// safe since Teensy 3 has its own isolated CAN1/CAN2 wiring, but confusing
// to read). The physical ODrives on this Teensy must be reconfigured to
// node_id 10-13 via odrivetool before this firmware will talk to them
// correctly — see Param.h's node ID block.
//
// Generated from teensy2/teensy2.ino (Teensy 2, right leg) with these changes:
//   - staticIP → 10.176.32.35
//   - teensy_udp_port_listening / PC_udp_port_listening → 8002
//   - ODrive objects and user data → odrv10..13 (ODRV10-13 macros, Param.h)
//   - sys_data_ initialised with N_ODRIVE_CAN6, N_ODRIVE_CAN7
//   - CAN1 mailbox filters recomputed for node IDs 10, 11

#define TEENSY_4_1

#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <QNEthernet.h>
#include "Command.h"
#include "DataContainer.h"
#include "Param.h"
#include "Utils.h"

#define CAN_BAUDRATE 250000
#define HEARTBEAT_MSG_RATE_MS 100 // 10 Hz
#define ENCODER_MSG_RATE_MS 2     // 500 Hz
#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

// ----- Network config (left arm) -----
IPAddress staticIP(10, 176, 32, 35);
IPAddress subnetMask(255, 255, 255, 0);
IPAddress gateway(10, 176, 32, 1);

constexpr uint32_t kDHCPTimeout = 15000;
constexpr uint16_t teensy_udp_port_listening = 8002;
constexpr uint16_t PC_udp_port_listening = 8002;

// PC address is learned from the first incoming UDP packet.
IPAddress pc_ip_;
bool pc_ip_known_ = false;

EthernetUDP udp;
bool first_packet_recv = false;

// Single shared mode variable across all command cases.
static uint8_t current_mode = 0;

// Timing / diagnostics
uint32_t loop_count = 0;
double sum_loop_duration_mcs = 0;
double sum_time_mcs_parse_udp_msg = 0;
double sum_time_mcs_send_CAN_command = 0;
double sum_time_mcs_send_udp_msg = 0;
double prev_time_mcs = 0;
elapsedMillis print_timer;

// Comms-loss watchdog: time since the last CRC-valid command. Reset on
// every confirmed-valid PositionCommand/VelocityCommand/TorqueCommand/
// IdleCommand/StartCommand, and once when first_packet_recv first becomes
// true. See the watchdog_tripped check in loop().
elapsedMillis last_valid_cmd_timer;
static bool watchdog_tripped = false;

// Static payload buffer — avoids heap allocation at 500 Hz.
static uint8_t payload_buf[MAX_CMD_PAYLOAD_SIZE];

// ----- ODrive objects (left arm: node_id 10-13) -----
ODriveCAN odrv10(wrap_can_intf(ODRV10_CAN), ODRV10_CAN_NODE_ID);
ODriveCAN odrv11(wrap_can_intf(ODRV11_CAN), ODRV11_CAN_NODE_ID);
ODriveCAN odrv12(wrap_can_intf(ODRV12_CAN), ODRV12_CAN_NODE_ID);
ODriveCAN odrv13(wrap_can_intf(ODRV13_CAN), ODRV13_CAN_NODE_ID);

ODriveCAN* odrives[]      = {&odrv10, &odrv11, &odrv12, &odrv13};
ODriveCAN* odrives_can1[] = {&odrv10, &odrv11};
ODriveCAN* odrives_can2[] = {&odrv12, &odrv13};

// Per-joint safety limits, indexed to match odrives[] above. Firmware-level
// backstop — enforced independently of the PC-side clamp in Leg.cpp, so a
// bad/garbled command still can't drive the hardware out of range. See
// Param.h for values and derivation.
static constexpr float q_min_turns[]   = {ODRV10_Q_MIN_TURNS, ODRV11_Q_MIN_TURNS, ODRV12_Q_MIN_TURNS, ODRV13_Q_MIN_TURNS};
static constexpr float q_max_turns[]   = {ODRV10_Q_MAX_TURNS, ODRV11_Q_MAX_TURNS, ODRV12_Q_MAX_TURNS, ODRV13_Q_MAX_TURNS};
static constexpr float tau_max_nm[]    = {ODRV10_TAU_MAX_NM, ODRV11_TAU_MAX_NM, ODRV12_TAU_MAX_NM, ODRV13_TAU_MAX_NM};
static constexpr float vel_max_turns_s[] = {ODRV10_VEL_MAX_TURNS_S, ODRV11_VEL_MAX_TURNS_S, ODRV12_VEL_MAX_TURNS_S, ODRV13_VEL_MAX_TURNS_S};

std::unique_ptr<SystemDataContainer> sys_data_;
size_t num_odrives = 0;
size_t num_odrives_data = 0;

struct ODriveUserData {
    ODriveUserData(int bus_idx, int node_idx, FlexCAN_T4_Base* can_ptr)
        : bus_idx_(bus_idx), node_idx_(node_idx), can_ptr_(can_ptr) {}

    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    bool is_active = false;  // set true if heartbeat received during setup
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    int bus_idx_;
    int node_idx_;
    FlexCAN_T4_Base* can_ptr_;
};

ODriveUserData odrv10_user_data(ODRV10_CAN_BUS_ID, ODRV10_CAN_ORDER_ID, &ODRV10_CAN);
ODriveUserData odrv11_user_data(ODRV11_CAN_BUS_ID, ODRV11_CAN_ORDER_ID, &ODRV11_CAN);
ODriveUserData odrv12_user_data(ODRV12_CAN_BUS_ID, ODRV12_CAN_ORDER_ID, &ODRV12_CAN);
ODriveUserData odrv13_user_data(ODRV13_CAN_BUS_ID, ODRV13_CAN_ORDER_ID, &ODRV13_CAN);

ODriveUserData* odrives_data[] = {
    &odrv10_user_data, &odrv11_user_data,
    &odrv12_user_data, &odrv13_user_data
};

// Real CAN node_id per odrives[] slot (10,11,12,13) — used only for console
// prints; now numerically identical to the local variable/array names
// (odrv10..13) since the 2026-09-02 relabel.
static constexpr int node_ids[] = {ODRV10_CAN_NODE_ID, ODRV11_CAN_NODE_ID, ODRV12_CAN_NODE_ID, ODRV13_CAN_NODE_ID};

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    uint32_t prev_error = odrv_user_data->last_heartbeat.Axis_Error;
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;

    static int heartbeat_count = 0;
    if (++heartbeat_count % 5000 == 0) {
        if (msg.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            Serial.print("WARNING: ODrive bus=");
            Serial.print(odrv_user_data->bus_idx_);
            Serial.print(" node=");
            Serial.print(odrv_user_data->node_idx_);
            Serial.print(" NOT in closed loop! State=");
            Serial.println(msg.Axis_State);
        }
    }

    // Edge-triggered (checked every heartbeat, not just once per 5000): a
    // fault that raises and clears within one 5000-heartbeat window would
    // otherwise never get printed at all.
    if (msg.Axis_Error != prev_error) {
        Serial.print(msg.Axis_Error != 0 ? "ERROR: " : "CLEARED: ");
        Serial.print("ODrive bus=");
        Serial.print(odrv_user_data->bus_idx_);
        Serial.print(" node=");
        Serial.print(odrv_user_data->node_idx_);
        Serial.print(" error code: 0x");
        Serial.println(msg.Axis_Error, HEX);
    }
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
    sys_data_->setEncoderEstimateAtBusAndNode(
        msg.Pos_Estimate, msg.Vel_Estimate,
        odrv_user_data->bus_idx_, odrv_user_data->node_idx_);

    static int feedback_count = 0;
    if (++feedback_count <= 10) {
        Serial.print("FB: bus=");
        Serial.print(odrv_user_data->bus_idx_);
        Serial.print(" node=");
        Serial.print(odrv_user_data->node_idx_);
        Serial.print(" pos=");
        Serial.print(msg.Pos_Estimate);
        Serial.print(" vel=");
        Serial.println(msg.Vel_Estimate);
    }
}

void onCanMessage1(const CanMsg& msg) {
    for (auto odrive: odrives_can1) { onReceive(msg, *odrive); }
}

void onCanMessage2(const CanMsg& msg) {
    for (auto odrive: odrives_can2) { onReceive(msg, *odrive); }
}

// ===== Setup =====

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < 30 && !Serial; ++i) { delay(100); }
    delay(200);

    if (!setupEthernetWithStaticIP()) {
        Serial.println("Ethernet failed to initialize");
        while (true);
    }
    delay(2000);

    sys_data_ = std::make_unique<SystemDataContainer>();
    sys_data_->add(SystemData<N_ODRIVE_CAN6>());  // left arm CAN1: l_shoulder_pitch + l_shoulder_roll
    sys_data_->add(SystemData<N_ODRIVE_CAN7>());  // left arm CAN2: l_shoulder_yaw + l_elbow

    num_odrives      = sizeof(odrives)      / sizeof(odrives[0]);
    num_odrives_data = sizeof(odrives_data) / sizeof(odrives_data[0]);
    if (num_odrives != num_odrives_data) {
        Serial.println("Error: num_odrives != num_odrives_data");
        while (true);
    }

    for (size_t i = 0; i < num_odrives; ++i) {
        odrives[i]->onFeedback(onFeedback, odrives_data[i]);
        odrives[i]->onStatus(onHeartbeat, odrives_data[i]);
    }

    if (!setupCAN()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    for (auto odrive: odrives) {
        while (!odrive->setEndpoint(274, HEARTBEAT_MSG_RATE_MS)) { delay(10); }
        while (!odrive->setEndpoint(275, ENCODER_MSG_RATE_MS))   { delay(10); }
        while (!odrive->setEndpoint(276, 0)) { delay(10); } // disable iq_msg
        while (!odrive->setEndpoint(277, 0)) { delay(10); } // disable error_msg
        while (!odrive->setEndpoint(278, 0)) { delay(10); } // disable temperature_msg
        while (!odrive->setEndpoint(279, 0)) { delay(10); } // disable bus_voltage_msg
        while (!odrive->setEndpoint(280, 0)) { delay(10); } // disable torques_msg
    }

    for (size_t i = 0; i < num_odrives; ++i) {
        for (int j = 0; j < 5; ++j) {
            delay(10);
            pumpEvents(*odrives_data[i]->can_ptr_);
        }
    }

    Serial.print("Found ODrives: ");
    for (size_t i = 0; i < num_odrives; ++i) {
        uint32_t deadline = millis() + 5000;
        while (!odrives_data[i]->received_heartbeat && millis() < deadline) {
            pumpEvents(*odrives_data[i]->can_ptr_);
            delay(1);
        }
        if (!odrives_data[i]->received_heartbeat) {
            Serial.print(" [node"); Serial.print(node_ids[i]); Serial.print(" TIMEOUT!]");
        } else {
            Serial.print(" [node"); Serial.print(node_ids[i]); Serial.print("]");
        }
    }
    Serial.println("");

    // Mark which ODrives responded — only these are used for feedback gating
    int active_count = 0;
    for (size_t i = 0; i < num_odrives; ++i) {
        odrives_data[i]->is_active = odrives_data[i]->received_heartbeat;
        if (odrives_data[i]->is_active) active_count++;
    }
    Serial.print("Active ODrives: "); Serial.println(active_count);

    Serial.println("ODrives found. Ready for commands.");
    Serial.println("Run './closed_loop_test --start' on PC to enable closed-loop control.");

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("PC<UDP>Teensy3<CAN>ODrivePro setup is complete.");
}

bool setupEthernetWithStaticIP()
{
    Ethernet.onLinkState([](bool state) {
        digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
    });

    if (!Ethernet.begin(staticIP, subnetMask, gateway)) { return false; }

    IPAddress ip = Ethernet.localIP();
    printf("%u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);

    if (!udp.begin(teensy_udp_port_listening)) {
        printf("Failed udp.begin\n");
        return false;
    }
    return true;
}

bool setupCAN()
{
    // CAN1: manual mailbox assignment for arm node IDs 10–11
    // CAN frame ID = (node_id << 5) | cmd_id
    //   node 10 encoder (cmd 0x09): 10*32+9  = 0x149
    //   node 11 encoder (cmd 0x09): 11*32+9  = 0x169
    //   node 10 heartbeat (cmd 0x01): 10*32+1 = 0x141
    //   node 11 heartbeat (cmd 0x01): 11*32+1 = 0x161
    //   node 10 cmd 0x05: 10*32+5 = 0x145
    //   node 11 cmd 0x05: 11*32+5 = 0x165
    can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
    can1.setMaxMB(20);

    // High-frequency mailboxes (2 per ID for buffering)
    uint16_t highFreqIDs[] = { 0x149, 0x169 };
    int mb = 0;
    for (int i = 0; i < 2; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, highFreqIDs[i]); mb++;
        can1.setMB(mb, RX); can1.setMBFilter(mb, highFreqIDs[i]); mb++;
    }

    // Lower-frequency mailboxes
    uint16_t lowFreqIDs[] = { 0x141, 0x161, 0x145, 0x165 };
    for (int i = 0; i < 4; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, lowFreqIDs[i]); mb++;
    }

    // Wildcard fallback mailboxes
    for (int i = 0; i < 4; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, 0x000); mb++;
    }
    for (int i = 0; i < 8; i++) { can1.setMB(mb, TX); mb++; }

    can1.enableMBInterrupts();
    can1.onReceive(onCanMessage1);
    can1.setClock(CLK_60MHz);

    // CAN2: auto-distribute for arm node IDs 12–13
    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);
    can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can2.enableMBInterrupts();
    can2.onReceive(onCanMessage2);
    can2.distribute();
    can2.setClock(CLK_60MHz);

    return true;
}

// ===== Main loop =====

void loop()
{
    pumpEvents(can1);
    pumpEvents(can2);

    // Serial Plotter feed (Tools > Serial Plotter) — 50 Hz is plenty for a
    // human-readable plot and keeps Serial overhead from perturbing loop
    // timing. Labeled "b{bus}n{node}_..." to match the bus/node identifiers
    // already used in the warning/error prints above.
    static elapsedMillis plot_timer;
    if (plot_timer >= 20) {
        plot_timer = 0;
        for (size_t i = 0; i < num_odrives_data; ++i) {
            ODriveUserData* d = odrives_data[i];
            Serial.print("b"); Serial.print(d->bus_idx_);
            Serial.print("n"); Serial.print(d->node_idx_);
            Serial.print("_pos:"); Serial.print(d->last_feedback.Pos_Estimate, 4);
            Serial.print(" b"); Serial.print(d->bus_idx_);
            Serial.print("n"); Serial.print(d->node_idx_);
            Serial.print("_fault:"); Serial.print(d->last_heartbeat.Axis_Error != 0 ? 1 : 0);
            Serial.print(" ");
        }
        Serial.println();
    }

    parseAndProcessUDPPacket();
    if (!first_packet_recv) return;

    // Comms-loss watchdog: if no CRC-valid command has arrived in
    // WATCHDOG_TIMEOUT_MS, stop trusting whatever the ODrives were last
    // told and idle them. Requires an explicit StartCommand to resume —
    // see the watchdog_tripped reset in the StartCommand handler.
    if (!watchdog_tripped && last_valid_cmd_timer > WATCHDOG_TIMEOUT_MS) {
        Serial.println("WATCHDOG: no valid command received in time - idling all ODrives");
        idleAllODrives();
        watchdog_tripped = true;
    }

    unsigned long start_time_mcs = micros();
    sendUDPPacket();
    sum_time_mcs_send_udp_msg += micros() - start_time_mcs;

    unsigned long current_time_mcs = micros();
    sum_loop_duration_mcs += current_time_mcs - prev_time_mcs;
    prev_time_mcs = current_time_mcs;
    loop_count++;

    // Periodic diagnostics every 5 seconds
    if (print_timer >= 5000) {
        if (loop_count > 0) {
            Serial.print("Loop: avg=");
            Serial.print(sum_loop_duration_mcs / loop_count);
            Serial.print("us | parse_udp=");
            Serial.print(sum_time_mcs_parse_udp_msg / loop_count);
            Serial.print("us | send_CAN=");
            Serial.print(sum_time_mcs_send_CAN_command / loop_count);
            Serial.print("us | send_udp=");
            Serial.print(sum_time_mcs_send_udp_msg / loop_count);
            Serial.print("us | count=");
            Serial.println(loop_count);
        }
        loop_count = 0;
        sum_loop_duration_mcs = 0;
        sum_time_mcs_parse_udp_msg = 0;
        sum_time_mcs_send_CAN_command = 0;
        sum_time_mcs_send_udp_msg = 0;
        print_timer = 0;
    }
}

// ===== UDP command parsing =====

void parseAndProcessUDPPacket()
{
    unsigned long start_time_mcs = micros();
    int size = udp.parsePacket();
    sum_time_mcs_parse_udp_msg += micros() - start_time_mcs;

    start_time_mcs = micros();
    if (size >= 0) {
        const uint8_t* data = udp.data();

        // Learn PC IP from the first incoming packet
        if (!pc_ip_known_) {
            pc_ip_ = udp.remoteIP();
            pc_ip_known_ = true;
            Serial.print("PC IP learned: ");
            Serial.println(pc_ip_);
        }

        if (!first_packet_recv) {
            first_packet_recv = true;
            last_valid_cmd_timer = 0;  // baseline from first contact, not Teensy boot
        }

        MsgType type = static_cast<MsgType>(*data);

        switch (type) {

        case MsgType::PositionCommand: {
            if (current_mode != 3) {
                for (size_t i = 0; i < num_odrives; ++i)
                    odrives[i]->setControllerMode(3, 1);
                current_mode = 3;
            }

            uint8_t num_motors = data[1];
            size_t payload_size = sizeof(uint8_t)
                + num_motors * sizeof(Input_Pos_TYPE)
                + num_motors * sizeof(Vel_FF_TYPE)
                + num_motors * sizeof(Torque_FF_TYPE);

            uint8_t received_crc   = data[1 + payload_size];
            uint8_t calculated_crc = calculate_crc8(data, 1 + payload_size);
            if (received_crc != calculated_crc) {
                Serial.println("CRC MISMATCH: PositionCommand dropped");
                break;
            }
            last_valid_cmd_timer = 0;

            memcpy(payload_buf, data + 1, payload_size);
            PositionCommand cmd;
            std::vector<uint8_t> payload(payload_buf, payload_buf + payload_size);
            cmd.deserialize(payload);

            // Safety clamp: bound each commanded position and torque
            // feedforward to this joint's limits before it ever reaches the
            // ODrive, independent of whatever the PC sent. Torque_FF is
            // motor-shaft Nm (same convention as TorqueCommand's clamp below).
            for (size_t i = 0; i < num_odrives && i < 4; ++i) {
                cmd.Input_Pos[i] = clampf(cmd.Input_Pos[i], q_min_turns[i], q_max_turns[i]);
                cmd.Torque_FF[i] = clampf(cmd.Torque_FF[i], -tau_max_nm[i], tau_max_nm[i]);
            }

            for (size_t i = 0; i < num_odrives; ++i)
                odrives[i]->setPosition(cmd.Input_Pos[i], cmd.Vel_FF[i], cmd.Torque_FF[i]);
            break;
        }

        case MsgType::VelocityCommand: {
            if (current_mode != 2) {
                for (size_t i = 0; i < num_odrives; ++i)
                    odrives[i]->setControllerMode(2, 1);
                current_mode = 2;
            }

            uint8_t num_motors = data[1];
            size_t payload_size = sizeof(uint8_t)
                + num_motors * sizeof(Input_Vel_TYPE)
                + sizeof(Input_Torque_FF_TYPE);

            uint8_t received_crc   = data[1 + payload_size];
            uint8_t calculated_crc = calculate_crc8(data, 1 + payload_size);
            if (received_crc != calculated_crc) {
                Serial.println("CRC MISMATCH: VelocityCommand dropped");
                break;
            }
            last_valid_cmd_timer = 0;

            memcpy(payload_buf, data + 1, payload_size);
            VelocityCommand cmd;
            std::vector<uint8_t> payload(payload_buf, payload_buf + payload_size);
            cmd.deserialize(payload);

            for (size_t i = 0; i < num_odrives && i < 4; ++i)
                cmd.Input_Vel[i] = clampf(cmd.Input_Vel[i], -vel_max_turns_s[i], vel_max_turns_s[i]);

            for (size_t i = 0; i < num_odrives; ++i)
                odrives[i]->setVelocity(cmd.Input_Vel[i], cmd.Input_Torque_FF);
            break;
        }

        case MsgType::TorqueCommand: {
            if (current_mode != 1) {
                for (size_t i = 0; i < num_odrives; ++i)
                    odrives[i]->setControllerMode(1, 1);
                current_mode = 1;
            }

            uint8_t num_motors = data[1];
            size_t payload_size = sizeof(uint8_t)
                + num_motors * sizeof(Input_Torque_TYPE);

            uint8_t received_crc   = data[1 + payload_size];
            uint8_t calculated_crc = calculate_crc8(data, 1 + payload_size);
            if (received_crc != calculated_crc) {
                Serial.println("CRC MISMATCH: TorqueCommand dropped");
                break;
            }
            last_valid_cmd_timer = 0;

            memcpy(payload_buf, data + 1, payload_size);
            TorqueCommand cmd;
            std::vector<uint8_t> payload(payload_buf, payload_buf + payload_size);
            cmd.deserialize(payload);

            // Torque is the most safety-critical of the three: nothing else
            // protects against a bad value here, so this clamp is the last
            // line of defense before current is commanded.
            for (size_t i = 0; i < num_odrives && i < 4; ++i)
                cmd.Input_Torque[i] = clampf(cmd.Input_Torque[i], -tau_max_nm[i], tau_max_nm[i]);

            for (size_t i = 0; i < num_odrives; ++i)
                odrives[i]->setTorque(cmd.Input_Torque[i]);
            break;
        }

        case MsgType::IdleCommand: {
            last_valid_cmd_timer = 0;
            Serial.println("Received IDLE command - putting all ODrives into IDLE state");
            idleAllODrives();
            Serial.println("All ODrives are now IDLE");
            break;
        }

        case MsgType::StartCommand: {
            Serial.println("Received START command - putting all ODrives into CLOSED_LOOP_CONTROL");
            // clearErrors()/setState() return whether the local CAN
            // peripheral accepted the message into a TX mailbox — NOT
            // whether the ODrive received/acted on it (see ODriveCAN.h's
            // doc comment). This was previously discarded everywhere in
            // this codebase; checking it here narrows "node never went
            // green" down to "the frame never left the Teensy" vs.
            // "the ODrive received it but didn't act" if it prints false.
            for (size_t i = 0; i < num_odrives; ++i) {
                bool clear_ok = odrives[i]->clearErrors();
                delay(1);
                bool state_ok = odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                delay(10);
                if (!clear_ok || !state_ok) {
                    Serial.print("WARNING: node "); Serial.print(node_ids[i]);
                    Serial.print(" TX not accepted (clearErrors=");
                    Serial.print(clear_ok); Serial.print(", setState=");
                    Serial.print(state_ok); Serial.println(")");
                }
            }
            Serial.println("All ODrives are now in CLOSED_LOOP_CONTROL");
            // Reset here, not at the top of this case: the loop above blocks
            // for tens of ms, and resetting before that work would risk the
            // watchdog self-tripping right after every StartCommand.
            last_valid_cmd_timer = 0;
            watchdog_tripped = false;  // explicit re-arm, per design: no auto-resume
            break;
        }

        case MsgType::Heartbeat: {
            // Deliberately does nothing else: no setControllerMode(), no
            // setState(), no ODrive call of any kind. Its only job is
            // satisfying the comms-loss watchdog during a settling wait
            // without implying or switching to any control mode.
            last_valid_cmd_timer = 0;
            break;
        }

        case MsgType::SetGains: {
            uint8_t num_motors = data[1];
            size_t payload_size = sizeof(uint8_t)
                + num_motors * sizeof(Pos_Gain_TYPE)
                + num_motors * sizeof(Vel_Gain_TYPE)
                + num_motors * sizeof(Vel_Integrator_Gain_TYPE);

            uint8_t received_crc   = data[1 + payload_size];
            uint8_t calculated_crc = calculate_crc8(data, 1 + payload_size);
            if (received_crc != calculated_crc) {
                Serial.println("CRC MISMATCH: SetGainsCommand dropped");
                break;
            }
            last_valid_cmd_timer = 0;

            memcpy(payload_buf, data + 1, payload_size);
            SetGainsCommand cmd;
            std::vector<uint8_t> payload(payload_buf, payload_buf + payload_size);
            cmd.deserialize(payload);

            // Negative gains are never correct regardless of tuning — this
            // is a sign-sanity check, not a magnitude limit (no data exists
            // to derive a reasonable ceiling, unlike the position/torque
            // clamps above).
            //
            // setPosGain()/setVelGains() return whether the local CAN
            // peripheral accepted the message into a TX mailbox, same as
            // setState() in StartCommand above — this loop previously fired
            // 8 sends (2 per node x 4 nodes) back-to-back with no delay and
            // discarded the result, the same bug idleAllODrives() had.
            // Retry + inter-node delay + a WARNING print closes that gap.
            for (size_t i = 0; i < num_odrives && i < 4; ++i) {
                float pg = cmd.Pos_Gain[i] < 0 ? 0.0f : cmd.Pos_Gain[i];
                float vg = cmd.Vel_Gain[i] < 0 ? 0.0f : cmd.Vel_Gain[i];
                float vig = cmd.Vel_Integrator_Gain[i] < 0 ? 0.0f : cmd.Vel_Integrator_Gain[i];

                bool pg_ok = false;
                for (int attempt = 0; attempt < 5 && !pg_ok; ++attempt) {
                    pg_ok = odrives[i]->setPosGain(pg);
                    if (!pg_ok) delay(2);
                }
                if (!pg_ok) {
                    Serial.print("WARNING: setPosGain TX not accepted after retries for node ");
                    Serial.println(node_ids[i]);
                }

                bool vg_ok = false;
                for (int attempt = 0; attempt < 5 && !vg_ok; ++attempt) {
                    vg_ok = odrives[i]->setVelGains(vg, vig);
                    if (!vg_ok) delay(2);
                }
                if (!vg_ok) {
                    Serial.print("WARNING: setVelGains TX not accepted after retries for node ");
                    Serial.println(node_ids[i]);
                }

                delay(10); // let the bus drain before the next node's gain-set
            }
            Serial.println("Applied SetGainsCommand");
            break;
        }

        default:
            Serial.println("Unknown MsgType!");
            break;
        }
    }
    sum_time_mcs_send_CAN_command += micros() - start_time_mcs;
}

// Shared by the IdleCommand handler and the comms-loss watchdog in loop().
void idleAllODrives()
{
    // setState() returns whether the local CAN peripheral accepted the
    // message into a TX mailbox (ODriveFlexCAN.hpp's sendMsg() maps
    // FlexCAN's write()==1 to true, write()==-1 "queued, no mailbox free
    // right now" to false) — NOT whether the ODrive received/acted on it
    // (see ODriveCAN.h's own doc comment). Unlike StartCommand's handler
    // below (delay(1)/delay(10) between each node's sends), this loop
    // previously fired all 4 setState() calls back-to-back with no delay —
    // if a TX mailbox wasn't immediately free for a given node, that node's
    // idle command silently never left the Teensy. Retry + inter-node delay
    // here closes that gap; the WARNING print still fires if a node
    // persistently can't get a message out after retrying.
    for (size_t i = 0; i < num_odrives; ++i) {
        bool ok = false;
        for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
            ok = odrives[i]->setState(ODriveAxisState::AXIS_STATE_IDLE);
            if (!ok) delay(2);
        }
        if (!ok) {
            Serial.print("WARNING: setState(IDLE) TX not accepted after retries for node ");
            Serial.println(node_ids[i]);
        }
        delay(10); // let the bus drain this node's frame before the next setState() fires
    }
    current_mode = 0; // reset so next mode always re-sends setControllerMode
}

bool receivedFeedbackOnAllODrives()
{
    bool any_active = false;
    for (size_t i = 0; i < num_odrives; ++i) {
        if (!odrives_data[i]->is_active) continue;
        any_active = true;
        if (!odrives_data[i]->received_feedback) return false;
    }
    return any_active;  // false if no active ODrives (don't send empty packets)
}

void resetODriveData()
{
    for (size_t i = 0; i < num_odrives; ++i)
        if (odrives_data[i]->is_active) odrives_data[i]->received_feedback = false;
}

// ===== UDP feedback sender =====

void sendUDPPacket()
{
    static int send_count = 0;
    static int check_count = 0;

    if (++check_count % 500 == 0) {
        Serial.print("UDP check: sent=");
        Serial.print(send_count);
        Serial.print(" feedback=[");
        for (size_t i = 0; i < num_odrives; ++i)
            Serial.print(odrives_data[i]->received_feedback ? "1" : "0");
        Serial.println("]");
    }

    if (!receivedFeedbackOnAllODrives()) return;
    resetODriveData();

    if (!pc_ip_known_) return;

    uint8_t buffer[sys_data_->dataSize()];
    sys_data_->serialize(buffer);

    if (!udp.send(pc_ip_, PC_udp_port_listening, buffer, sizeof(buffer))) {
        printf("Error sending udp from Teensy3\n");
    } else {
        send_count++;
        if (send_count <= 5 || send_count % 100 == 0) {
            Serial.print("Sent UDP #");
            Serial.println(send_count);
        }
    }
}
