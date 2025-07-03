#include <iostream>
#include <cstring>
#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <QNEthernet.h>
#include "Command.h"
#include "Data.h"

#define CAN_BAUDRATE 250000 // CAN Simple can go up to 1e6?
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

IPAddress staticIP{10, 176, 32, 33};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{10, 176, 32, 1};

constexpr uint32_t kDHCPTimeout = 15000; // 15 seconds
constexpr uint16_t teensy_udp_port_listening = 8000;
constexpr uint16_t PC_udp_port_listening = 8000;
constexpr int MAX_NODES = 1;             // Maximum number of nodes
constexpr int MAX_NUM_SAMPLES = 5000;

EthernetUDP udp;

uint8_t can_data[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command[MAX_NODES][8]; // CAN command buffer for each node

uint8_t can_data_bus2[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command_bus2[MAX_NODES][8]; // CAN command buffer for each node

unsigned long last_packet_time_bus1[MAX_NODES] = {0};
unsigned long total_latency_bus1[MAX_NODES] = {0};
unsigned int packet_count_bus1[MAX_NODES] = {0};

unsigned long last_packet_time_bus2[MAX_NODES] = {0};
unsigned long total_latency_bus2[MAX_NODES] = {0};
unsigned int packet_count_bus2[MAX_NODES] = {0};

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

unsigned long CAN_udp_msg_parse_time_mcs = 0.;
unsigned long CAN_udp_msg_process_time_mcs = 0.;
unsigned long CAN_udp_msg_send_time_mcs = 0.;
unsigned long CAN_total_duration_mcs = 0.;
unsigned long prev_time_mcs = 0.;

// Instantiate ODrive objects // need to be declared early for fromBuffer()
ODriveCAN odrv0(wrap_can_intf(can1), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can1), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can1), ODRV2_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2}; // Make sure all ODriveCAN instances are accounted for here

template<typename Func, typename Tuple>
void odriveCommandWrapper(Func&& f, Tuple&& args) {
    std::apply(std::forward<Func>(f), std::forward<Tuple>(args));
}

// Calculate CRC-8 checksum
// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;
uint8_t calculate_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void printIPAddress()
{
    IPAddress ip = Ethernet.localIP();
    printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.broadcastIP();
    printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

std::unique_ptr<CommandBase> sys_command_;
std::unique_ptr<SystemData> sys_data_;

struct ODriveUserData {

  ODriveUserData(int idx)
  {
    idx_ = idx;
  }

  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
  int idx_;

};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data(0);
ODriveUserData odrv1_user_data(1);
ODriveUserData odrv2_user_data(2);

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

struct FeedbackContainer {
    ODriveUserData* odrv_user_data;
    int idx;
};

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  /*FeedbackContainer* container = static_cast<FeedbackContainer*>(user_data);
  ODriveUserData* odrv_user_data = container->odrv_user_data;
  int idx = container->idx;
  //odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
  sys_data_->encoder_Pos_Estimate[idx] = msg.Pos_Estimate;
  sys_data_->encoder_Vel_Estimate[idx] = msg.Vel_Estimate;*/
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->received_feedback = true;
  sys_data_->encoder_Pos_Estimate[odrv_user_data->idx_] = msg.Pos_Estimate;
  sys_data_->encoder_Vel_Estimate[odrv_user_data->idx_] = msg.Vel_Estimate;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

void setup()
{
    Serial.begin(115200);

    // Wait for up to 3 seconds for the serial port to be opened on the PC side.
    // If no PC connects, continue anyway.
    for (int i = 0; i < 30 && !Serial; ++i) {
      delay(100);
    }
    delay(200);

    if (!setupEthernetWithStaticIP()) {
      Serial.println("Ethernet failed to initialize");
      while (true); // spin indefinitely
    };
    delay(2000); // wait for Ethernet confirmation

    // Register callbacks for the heartbeat and encoder feedback messages
    sys_data_ = std::make_unique<SystemData>();
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv1.onFeedback(onFeedback, &odrv1_user_data);
    odrv1.onStatus(onHeartbeat, &odrv1_user_data);
    odrv2.onFeedback(onFeedback, &odrv2_user_data);
    odrv2.onStatus(onHeartbeat, &odrv2_user_data);

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCAN()) {
      Serial.println("CAN failed to initialize: reset required");
      while (true); // spin indefinitely
    }

    Serial.print("Found ODrives: ");
    while (!odrv0_user_data.received_heartbeat) {
      pumpEvents(can1);
      delay(100);
    }

    Serial.print(" [odrv0]");

    while (!odrv1_user_data.received_heartbeat) {
      pumpEvents(can1);
      delay(100);
    }
    Serial.print(" [odrv1]");

    while (!odrv2_user_data.received_heartbeat) {
      pumpEvents(can1);
      delay(100);
    }
    Serial.println(" [odrv2]");

    // request bus voltage and current (1sec timeout)
    Serial.print("Request bus voltage and current:");
    Get_Bus_Voltage_Current_msg_t vbus;
    while(!odrv0.request(vbus, 1)) {delay(100);}
    Serial.print(" [odrv0]");
    while(!odrv1.request(vbus, 1)) {delay(100);}
    Serial.print(" [odrv1]");
    while(!odrv2.request(vbus, 1)) {delay(100);}
    Serial.println(" [odrv2]");

    // print the last odrive's DC voltage and current
    Serial.print("DC voltage [V]: ");
    Serial.print(vbus.Bus_Voltage);
    Serial.print(" | DC current [A]: ");
    Serial.println(vbus.Bus_Current);
  
    // enabling closed loop control
    Serial.print("Enabling closed-loop control: ");
    while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrv0.clearErrors();
      delay(1);
      odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can1);
      }
    }
    Serial.print(" [odrv0]");

    while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrv1.clearErrors();
      delay(1);
      odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can1);
      }
    }
    Serial.print(" [odrv1]");

    while (odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrv2.clearErrors();
      delay(1);
      odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can1);
      }
    }
    Serial.println(" [odrv2]");
    
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("PC<UDP>Teensy<CAN>ODrivePro setup is complete.");
}

bool setupEthernetWithStaticIP()
{
    // Fetch MAC address out of the Teensy
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


    // Show whether a cable is plugged in or not
    Ethernet.onLinkState([](bool state)
    {
      if (state)
        digitalWrite(LED_BUILTIN, HIGH);
      else
        digitalWrite(LED_BUILTIN, LOW);

      printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
    });

    // Static IP
    printf("Starting Ethernet with static IP...\r\n");
    if (!Ethernet.begin(staticIP, subnetMask, gateway)) {
      printf("Failed to start Ethernet\r\n");
      return false;
    }

    printf("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    if (!udp.begin(teensy_udp_port_listening))
    {
      printf("Failed udp.begin\n");
      return false;
    }

    return true;
}

bool setupCAN()
{
    can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
    can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can1.setMBFilter(MB1, 0, 0x0);
    can1.enhanceFilter(MB1);
    can1.enableMBInterrupts();
    can1.onReceive(onCanMessage);
    
    can1.distribute();
    can1.setClock(CLK_60MHz);

    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);
    can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can2.setMBFilter(MB1, 0, 0x0);
    can2.enhanceFilter(MB1);
    can2.enableMBInterrupts();
    can2.onReceive(onCanMessage);

    can2.distribute();
    can2.setClock(CLK_60MHz);
    // can2.mailboxStatus();

    can3.begin();
    can3.setBaudRate(CAN_BAUDRATE);
    can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can3.setMBFilter(MB1, 0, 0x0);
    can3.enhanceFilter(MB1);
    can3.enableMBInterrupts();
    can3.onReceive(onCanMessage);

    can3.distribute();
    can3.setClock(CLK_60MHz);
    // can3.mailboxStatus();

    return true;
}


void loop()
{
    pumpEvents(can1);
    pumpEvents(can2); // This is required on some platforms to handle incoming feedback CAN messages
    pumpEvents(can3);

    parseAndProcessUDPPacket(); // receive UDP message from UP to Teensy
    if (!first_packet_recv)
        return;

    unsigned long start_time_mcs = micros();
    sendUDPPacket(); // send UDP message from Teensy to UP
    CAN_udp_msg_send_time_mcs = micros() - start_time_mcs;

    unsigned long current_time_mcs = micros();
    unsigned long loop_duration_mcs = current_time_mcs - prev_time_mcs;
    prev_time_mcs = current_time_mcs;
    /*Serial.print("[CAN loop timing] ");
    Serial.print("frequency: ");
    Serial.print(1000000/loop_duration_mcs);
    Serial.print(" Hz | ");
    Serial.print("parse time: ");
    Serial.print(CAN_udp_msg_parse_time_mcs);
    Serial.print(" mcs | ");
    Serial.print("process time: ");
    Serial.print(CAN_udp_msg_process_time_mcs);
    Serial.print(" mcs | ");
    Serial.print("send time: ");
    Serial.print(CAN_udp_msg_send_time_mcs);
    Serial.println(" mcs");*/
}

void parseAndProcessUDPPacket()
{
    unsigned long start_time_mcs = micros();
    int size = udp.parsePacket();
    CAN_udp_msg_parse_time_mcs = micros() - start_time_mcs;

    start_time_mcs = micros();
    if (size >= 0)
    {
        const uint8_t *data = udp.data();
        
        if (!first_packet_recv)
            first_packet_recv = true;

        const uint8_t msg_buffer = *data;
        MsgType type = static_cast<MsgType>(msg_buffer);
        //Serial.println(static_cast<uint8_t>(type));

        switch (type) {
        case MsgType::PositionCommand: {
            //Serial.println("MsgType::PositionCommand");
            //Serial.print("MsgType::Position ");
            PositionCommand cmd;
            std::vector<uint8_t> payload(data+1, data+cmd.dataSize()+2); // w/out byte corresponding to type
            // Calculate CRC-8 for the payload
            //uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());
            //const uint8_t received_crc = data[cmd.dataSize()+1];
            //if (received_crc == calculated_crc) {
            if (true) {
              cmd.deserialize(payload);
              //return std::make_unique<PositionCommand>(cmd);
              //cmd.printValue();
              odriveCommandWrapper([&](Input_Pos_TYPE p, Vel_FF_TYPE v_ff, Torque_FF_TYPE t_ff) { odrv0.setPosition(p, v_ff, t_ff); },
                    cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Pos_TYPE p, Vel_FF_TYPE v_ff, Torque_FF_TYPE t_ff) { odrv1.setPosition(p, v_ff, t_ff); },
                    cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Pos_TYPE p, Vel_FF_TYPE v_ff, Torque_FF_TYPE t_ff) { odrv2.setPosition(p, v_ff, t_ff); },
                    cmd.getCommandValue());
            }
            break;
        }
        case MsgType::VelocityCommand: {
            //Serial.print("MsgType::VelocityCommand ");
            //Serial.println(static_cast<uint8_t>(type));
            VelocityCommand cmd;
            std::vector<uint8_t> payload(data+1, data+cmd.dataSize()+2); // w/out byte corresponding to type
            // Calculate CRC-8 for the payload
            //uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());
            //const uint8_t received_crc = data[cmd.dataSize()+1];
            //if (received_crc == calculated_crc) {
            if (true) {
              cmd.deserialize(payload);
              //cmd.printValue();
              odriveCommandWrapper([&](Input_Vel_TYPE v, Input_Torque_FF_TYPE t_ff) { odrv0.setVelocity(v, t_ff); },
                    cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Vel_TYPE v, Input_Torque_FF_TYPE t_ff) { odrv1.setVelocity(v, t_ff); },
                    cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Vel_TYPE v, Input_Torque_FF_TYPE t_ff) { odrv2.setVelocity(v, t_ff); },
                    cmd.getCommandValue());
            }
            break;
        }
        case MsgType::TorqueCommand: {
            //Serial.print("MsgType::Torque ");
            //Serial.println(static_cast<uint8_t>(type));
            TorqueCommand cmd;
            std::vector<uint8_t> payload(data+1, data+cmd.dataSize()+2); // w/out byte corresponding to type
            // Calculate CRC-8 for the payload
            //uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());
            //const uint8_t received_crc = data[cmd.dataSize()+1];
            //if (received_crc == calculated_crc) {  
            if (true) {
              cmd.deserialize(payload);
              odriveCommandWrapper([&](Input_Torque_TYPE t) { odrv0.setTorque(t); },
                      cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Torque_TYPE t) { odrv1.setTorque(t); },
                      cmd.getCommandValue());
              odriveCommandWrapper([&](Input_Torque_TYPE t) { odrv2.setTorque(t); },
                      cmd.getCommandValue());
            }
            break;
        }
        default: {
            std::cerr << "Unknown MsgType!\n";
            break;
        }
        }
    }
    CAN_udp_msg_process_time_mcs = micros() - start_time_mcs;
}

void sendUDPPacket()
{
    if (odrv0_user_data.received_feedback && odrv1_user_data.received_feedback && odrv2_user_data.received_feedback) {
      //Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      //Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      odrv0_user_data.received_feedback = false;
      odrv1_user_data.received_feedback = false;
      odrv2_user_data.received_feedback = false;

      //SystemData packet(feedback.Pos_Estimate, feedback.Vel_Estimate);
      uint8_t buffer[sys_data_->dataSize()];
      //packet.serialize(buffer);
      sys_data_->writeToBuffer(buffer);

      if (!udp.send("10.176.32.14", PC_udp_port_listening, buffer, sizeof(buffer)))
      {
          printf("Error sending udp from Teensy\n");
      }
    }
}
