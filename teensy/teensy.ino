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
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// ----- Network config -----
IPAddress staticIP(10, 176, 32, 33);
IPAddress subnetMask(255, 255, 255, 0);
IPAddress gateway(10, 176, 32, 1);

constexpr uint32_t kDHCPTimeout = 15000;
constexpr uint16_t teensy_udp_port_listening = 8000;
constexpr uint16_t PC_udp_port_listening = 8000;

// A4: PC address is learned from the first incoming UDP packet.
// Avoids hardcoding an IP that requires a reflash to change.
IPAddress pc_ip_;
bool pc_ip_known_ = false;

EthernetUDP udp;
bool first_packet_recv = false;

// Deferred link-state info: set in the onLinkState callback (unsafe to call
// Ethernet.linkSpeed() / linkIsFullDuplex() there), printed from setup().
static volatile bool link_state_changed = false;
static volatile bool link_is_up = false;

// A2: Single shared mode variable across all command cases.
// Prevents stale per-case statics from skipping setControllerMode()
// calls when the user switches modes and switches back.
static uint8_t current_mode = 0;

// A8: Timing / diagnostics
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

// A7: Static payload buffer — avoids heap allocation at 500 Hz.
static uint8_t payload_buf[MAX_CMD_PAYLOAD_SIZE];

// ----- ODrive objects -----
ODriveCAN odrv0(wrap_can_intf(ODRV0_CAN), ODRV0_CAN_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(ODRV1_CAN), ODRV1_CAN_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(ODRV2_CAN), ODRV2_CAN_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(ODRV3_CAN), ODRV3_CAN_NODE_ID);
ODriveCAN odrv4(wrap_can_intf(ODRV4_CAN), ODRV4_CAN_NODE_ID);
//ODriveCAN odrv5(wrap_can_intf(ODRV5_CAN), ODRV5_CAN_NODE_ID);
//ODriveCAN odrv6(wrap_can_intf(ODRV6_CAN), ODRV6_CAN_NODE_ID);
//ODriveCAN odrv7(wrap_can_intf(ODRV7_CAN), ODRV7_CAN_NODE_ID);

ODriveCAN* odrives[]      = {&odrv0, &odrv1, &odrv2, &odrv3, &odrv4};
ODriveCAN* odrives_can1[] = {&odrv0, &odrv1};
// odrv4 (l_ankle) now shares the physical CAN2 wire with odrv2/odrv3 — see
// ODRV4_CAN in Param.h. CAN3 is currently unused on this Teensy.
ODriveCAN* odrives_can2[] = {&odrv2, &odrv3, &odrv4};

// Per-joint safety limits, indexed to match odrives[] above. Firmware-level
// backstop — enforced independently of the PC-side clamp in Leg.cpp, so a
// bad/garbled command still can't drive the hardware out of range. See
// Param.h for values and derivation.
static constexpr float q_min_turns[]   = {ODRV0_Q_MIN_TURNS, ODRV1_Q_MIN_TURNS, ODRV2_Q_MIN_TURNS, ODRV3_Q_MIN_TURNS, ODRV4_Q_MIN_TURNS};
static constexpr float q_max_turns[]   = {ODRV0_Q_MAX_TURNS, ODRV1_Q_MAX_TURNS, ODRV2_Q_MAX_TURNS, ODRV3_Q_MAX_TURNS, ODRV4_Q_MAX_TURNS};
static constexpr float tau_max_nm[]    = {ODRV0_TAU_MAX_NM, ODRV1_TAU_MAX_NM, ODRV2_TAU_MAX_NM, ODRV3_TAU_MAX_NM, ODRV4_TAU_MAX_NM};
static constexpr float vel_max_turns_s[] = {ODRV0_VEL_MAX_TURNS_S, ODRV1_VEL_MAX_TURNS_S, ODRV2_VEL_MAX_TURNS_S, ODRV3_VEL_MAX_TURNS_S, ODRV4_VEL_MAX_TURNS_S};

std::unique_ptr<SystemDataContainer> sys_data_;
size_t num_odrives = 0;
size_t num_odrives_data = 0;

struct ODriveUserData {
    ODriveUserData(int bus_idx, int node_idx, FlexCAN_T4_Base* can_ptr)
        : bus_idx_(bus_idx), node_idx_(node_idx), can_ptr_(can_ptr) {}

    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    int bus_idx_;
    int node_idx_;
    FlexCAN_T4_Base* can_ptr_;
};

ODriveUserData odrv0_user_data(ODRV0_CAN_BUS_ID, ODRV0_CAN_ORDER_ID, &ODRV0_CAN);
ODriveUserData odrv1_user_data(ODRV1_CAN_BUS_ID, ODRV1_CAN_ORDER_ID, &ODRV1_CAN);
ODriveUserData odrv2_user_data(ODRV2_CAN_BUS_ID, ODRV2_CAN_ORDER_ID, &ODRV2_CAN);
ODriveUserData odrv3_user_data(ODRV3_CAN_BUS_ID, ODRV3_CAN_ORDER_ID, &ODRV3_CAN);
ODriveUserData odrv4_user_data(ODRV4_CAN_BUS_ID, ODRV4_CAN_ORDER_ID, &ODRV4_CAN);
//ODriveUserData odrv5_user_data(ODRV5_CAN_BUS_ID, ODRV5_CAN_ORDER_ID, &ODRV5_CAN);
//ODriveUserData odrv6_user_data(ODRV6_CAN_BUS_ID, ODRV6_CAN_ORDER_ID, &ODRV6_CAN);
//ODriveUserData odrv7_user_data(ODRV7_CAN_BUS_ID, ODRV7_CAN_ORDER_ID, &ODRV7_CAN);

ODriveUserData* odrives_data[] = {
    &odrv0_user_data, &odrv1_user_data, &odrv2_user_data, &odrv3_user_data, &odrv4_user_data
};

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
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
        if (msg.Axis_Error != 0) {
            Serial.print("ERROR: ODrive bus=");
            Serial.print(odrv_user_data->bus_idx_);
            Serial.print(" node=");
            Serial.print(odrv_user_data->node_idx_);
            Serial.print(" has error code: 0x");
            Serial.println(msg.Axis_Error, HEX);
        }
    }
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
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

// CAN3 currently has no ODrives wired to it (l_ankle moved to CAN2) — no-op,
// kept so pumpEvents(can3) in loop() still has a registered handler.
void onCanMessage3(const CanMsg& msg) {
    (void)msg;
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

    // Print link speed/duplex here — safe to call Ethernet.linkSpeed() /
    // linkIsFullDuplex() outside the onLinkState callback.
    if (link_state_changed) {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("ms] Link state: ");
        Serial.print(link_is_up ? "UP" : "DOWN");
        if (link_is_up) {
            Serial.print(" (");
            Serial.print(Ethernet.linkSpeed());
            Serial.print("Mbps ");
            Serial.print(Ethernet.linkIsFullDuplex() ? "Full" : "Half");
            Serial.print(" Duplex)");
        }
        Serial.println();
        link_state_changed = false;
    }

    sys_data_ = std::make_unique<SystemDataContainer>();
    sys_data_->add(SystemData<N_ODRIVE_CAN1>());
    sys_data_->add(SystemData<N_ODRIVE_CAN2>());
    // CAN3 carries only l_ankle (1 real motor), but UPXtreme.cpp on the PC
    // side hardcodes SystemData<2> for every bus, so this bus is padded to
    // 2 slots to keep packet sizes in sync; node 1 is unused/always zero.
    sys_data_->add(SystemData<2>());

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

    // Configure ODrive message rates — skip silently if ODrive doesn't respond
    for (auto odrive: odrives) {
        uint32_t t;
        t = millis(); while (!odrive->setEndpoint(274, HEARTBEAT_MSG_RATE_MS) && millis()-t < 500) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(275, ENCODER_MSG_RATE_MS)   && millis()-t < 500) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(276, 0) && millis()-t < 200) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(277, 0) && millis()-t < 200) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(278, 0) && millis()-t < 200) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(279, 0) && millis()-t < 200) { delay(10); }
        t = millis(); while (!odrive->setEndpoint(280, 0) && millis()-t < 200) { delay(10); }
    }

    Serial.print("Found ODrives: ");
    for (size_t i = 0; i < num_odrives; ++i) {
        // Wait with a 5s timeout rather than spinning forever on a missing ODrive
        uint32_t deadline = millis() + 5000;
        while (!odrives_data[i]->received_heartbeat && millis() < deadline) {
            pumpEvents(*odrives_data[i]->can_ptr_);
            delay(1);
        }
        if (!odrives_data[i]->received_heartbeat) {
            Serial.print(" [odrv"); Serial.print(i); Serial.print(" TIMEOUT!]");
        } else {
            Serial.print(" [odrv"); Serial.print(i); Serial.print("]");
        }
    }
    Serial.println("");

    Serial.println("ODrives found. Ready for commands.");
    Serial.println("Run './closed_loop_test --start' on PC to enable closed-loop control.");

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("PC<UDP>Teensy<CAN>ODrivePro setup is complete.");
}

bool setupEthernetWithStaticIP()
{
    // Only do GPIO in the callback — calling Ethernet.linkSpeed() /
    // linkIsFullDuplex() here reads PHY registers over MDIO while the
    // ENET peripheral may not be fully initialised, which hangs the MAC.
    Ethernet.onLinkState([](bool state) {
        digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
        link_is_up = state;
        link_state_changed = true;
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
    // CAN1: manual mailbox assignment for left-leg node IDs 0–1
    can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
    can1.setMaxMB(20);

    // High-frequency IDs — 2 mailboxes each for buffering
    uint16_t highFreqIDs[] = { 0x009, 0x029, 0x049 };
    int mb = 0;
    for (int i = 0; i < 3; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, highFreqIDs[i]); mb++;
        can1.setMB(mb, RX); can1.setMBFilter(mb, highFreqIDs[i]); mb++;
    }

    // Lower-frequency IDs — 1 mailbox each
    uint16_t lowFreqIDs[] = { 0x001, 0x021, 0x041, 0x005, 0x025, 0x045 };
    for (int i = 0; i < 6; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, lowFreqIDs[i]); mb++;
    }

    // Wildcard fallback mailboxes
    for (int i = 0; i < 3; i++) {
        can1.setMB(mb, RX); can1.setMBFilter(mb, 0x000); mb++;
    }
    for (int i = 0; i < 5; i++) { can1.setMB(mb, TX); mb++; }

    can1.enableMBInterrupts();
    can1.onReceive(onCanMessage1);
    can1.setClock(CLK_60MHz);

    // CAN2: auto-distribute for left-leg node IDs 2–4 (l_hip_pitch, l_knee,
    // l_ankle — l_ankle moved here from CAN3, see ODRV4_CAN in Param.h)
    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);
    can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can2.enableMBInterrupts();
    can2.onReceive(onCanMessage2);
    can2.distribute();
    can2.setClock(CLK_60MHz);

    // CAN3: currently unused (no ODrives wired to it) — initialized for
    // completeness/future use only.
    can3.begin();
    can3.setBaudRate(CAN_BAUDRATE);
    can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can3.enableMBInterrupts();
    can3.onReceive(onCanMessage3);
    can3.distribute();
    can3.setClock(CLK_60MHz);

    return true;
}

// ===== Main loop =====

void loop()
{
    // Continuous link-state + alive heartbeat — independent of UDP traffic,
    // so we have visibility even if the PC never gets a packet through.
    if (link_state_changed) {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("ms] Link state changed: ");
        Serial.println(link_is_up ? "UP" : "DOWN");
        link_state_changed = false;
    }
    static elapsedMillis alive_timer;
    if (alive_timer >= 2000) {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("ms] alive | link=");
        Serial.print(Ethernet.linkState() ? "UP" : "DOWN");
        Serial.print(" | pc_ip_known=");
        Serial.print(pc_ip_known_ ? "yes" : "no");
        Serial.print(" | first_packet_recv=");
        Serial.println(first_packet_recv ? "yes" : "no");
        alive_timer = 0;
    }

    pumpEvents(can1);
    pumpEvents(can2);
    pumpEvents(can3);

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

    // A8: Periodic diagnostics — print stats every 5 seconds
    if (print_timer >= 5000) {
        double avg_loop = 0, avg_udp = 0, avg_can = 0;
        if (loop_count > 0) {
            avg_loop = sum_loop_duration_mcs / loop_count;
            avg_udp  = sum_time_mcs_send_udp_msg / loop_count;
            avg_can  = sum_time_mcs_send_CAN_command / loop_count;
            Serial.print("Loop: avg=");
            Serial.print(avg_loop);
            Serial.print("us | parse_udp=");
            Serial.print(sum_time_mcs_parse_udp_msg / loop_count);
            Serial.print("us | send_CAN=");
            Serial.print(avg_can);
            Serial.print("us | send_udp=");
            Serial.print(avg_udp);
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

        // A4: learn PC IP from the first incoming packet
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
            // A2: shared current_mode (not per-case static)
            if (current_mode != 3) {
                for (size_t i = 0; i < num_odrives; ++i)
                    odrives[i]->setControllerMode(3, 1);
                current_mode = 3;
            }

            uint8_t num_motors = data[1];
            size_t payload_size = sizeof(uint8_t)
                + num_motors * sizeof(Input_Pos_TYPE)
                + num_motors * sizeof(Vel_FF_TYPE)
                + sizeof(Torque_FF_TYPE);

            // A1: CRC check — packet layout: [type(1)] [payload(payload_size)] [crc(1)]
            // CRC covers type byte + payload, matching PC sendToTeensy().
            uint8_t received_crc   = data[1 + payload_size];
            uint8_t calculated_crc = calculate_crc8(data, 1 + payload_size);
            if (received_crc != calculated_crc) {
                Serial.println("CRC MISMATCH: PositionCommand dropped");
                break;
            }
            last_valid_cmd_timer = 0;

            memcpy(payload_buf, data + 1, payload_size); // A7: static buffer
            PositionCommand cmd;
            std::vector<uint8_t> payload(payload_buf, payload_buf + payload_size);
            cmd.deserialize(payload);

            // Safety clamp: bound each commanded position to this joint's
            // limits before it ever reaches the ODrive, independent of
            // whatever the PC sent.
            for (size_t i = 0; i < num_odrives && i < 5; ++i)
                cmd.Input_Pos[i] = clampf(cmd.Input_Pos[i], q_min_turns[i], q_max_turns[i]);

            // A5: loop instead of 4 hardcoded calls
            for (size_t i = 0; i < num_odrives; ++i)
                odrives[i]->setPosition(cmd.Input_Pos[i], cmd.Vel_FF[i], cmd.Torque_FF);
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

            for (size_t i = 0; i < num_odrives && i < 5; ++i)
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
            for (size_t i = 0; i < num_odrives && i < 5; ++i)
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
            for (size_t i = 0; i < num_odrives; ++i) {
                odrives[i]->clearErrors();
                delay(1);
                odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                delay(10);
            }

            // Pump CAN events so fresh heartbeats (reflecting the requested
            // state change) arrive before we report back what happened.
            for (int j = 0; j < 30; ++j) {
                delay(10);
                pumpEvents(can1);
                pumpEvents(can2);
                pumpEvents(can3);
            }

            for (size_t i = 0; i < num_odrives; ++i) {
                Serial.print("  odrv"); Serial.print(i);
                Serial.print(": state="); Serial.print(odrives_data[i]->last_heartbeat.Axis_State);
                Serial.print(" err=0x"); Serial.println(odrives_data[i]->last_heartbeat.Axis_Error, HEX);
            }
            Serial.println("All ODrives are now in CLOSED_LOOP_CONTROL");
            // Reset here, not at the top of this case: the pump loop above
            // blocks for ~300ms, which alone exceeds WATCHDOG_TIMEOUT_MS —
            // resetting before that work would let the watchdog self-trip
            // immediately after every StartCommand, regardless of the PC.
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

        default:
            PRINTLN("Unknown MsgType!");
            break;
        }
    }
    sum_time_mcs_send_CAN_command += micros() - start_time_mcs;
}

// Shared by the IdleCommand handler and the comms-loss watchdog in loop().
void idleAllODrives()
{
    for (size_t i = 0; i < num_odrives; ++i)
        odrives[i]->setState(ODriveAxisState::AXIS_STATE_IDLE);
    current_mode = 0; // reset so next mode always re-sends setControllerMode
}

bool receivedFeedbackOnAllODrives()
{
    for (size_t i = 0; i < num_odrives; ++i)
        if (!odrives_data[i]->received_feedback) return false;
    return true;
}

void resetODriveData()
{
    for (size_t i = 0; i < num_odrives; ++i)
        odrives_data[i]->received_feedback = false;
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

    // A4: use the learned PC IP, not a hardcoded address
    if (!pc_ip_known_) return;

    uint8_t buffer[sys_data_->dataSize()];
    sys_data_->serialize(buffer);

    if (!udp.send(pc_ip_, PC_udp_port_listening, buffer, sizeof(buffer))) {
        printf("Error sending udp from Teensy\n");
    } else {
        send_count++;
        if (send_count <= 5 || send_count % 100 == 0) {
            Serial.print("Sent UDP #");
            Serial.println(send_count);
        }
    }
}
