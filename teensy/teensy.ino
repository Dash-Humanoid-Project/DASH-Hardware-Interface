#define TEENSY_4_1

#include <cstring>
#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <QNEthernet.h>
#include "Command.h"
#include "DataContainer.h"
#include "Param.h"

#include <unordered_map>

#define CAN_BAUDRATE 250000 // CAN Simple can go up to 1e6?

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

bool first_packet_recv = false;

/*unsigned long CAN_udp_msg_parse_time_mcs = 0.;
unsigned long CAN_udp_msg_process_time_mcs = 0.;
unsigned long CAN_udp_msg_send_time_mcs = 0.;
unsigned long CAN_total_duration_mcs = 0.;
unsigned long prev_time_mcs = 0.;*/

// Instantiate ODrive objects // need to be declared early for fromBuffer()
ODriveCAN odrv0(wrap_can_intf(ODRV0_CAN), ODRV0_CAN_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(ODRV1_CAN), ODRV1_CAN_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(ODRV2_CAN), ODRV2_CAN_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(ODRV3_CAN), ODRV3_CAN_NODE_ID);
ODriveCAN odrv4(wrap_can_intf(ODRV4_CAN), ODRV4_CAN_NODE_ID);
ODriveCAN odrv5(wrap_can_intf(ODRV5_CAN), ODRV5_CAN_NODE_ID);
ODriveCAN odrv6(wrap_can_intf(ODRV6_CAN), ODRV6_CAN_NODE_ID);
ODriveCAN odrv7(wrap_can_intf(ODRV7_CAN), ODRV7_CAN_NODE_ID);
ODriveCAN odrv8(wrap_can_intf(ODRV8_CAN), ODRV8_CAN_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3, &odrv4, &odrv5, &odrv6, &odrv7, &odrv8}; // Make sure all ODriveCAN instances are accounted for here
ODriveCAN* odrives_can1[] = {&odrv0, &odrv1, &odrv2};
ODriveCAN* odrives_can2[] = {&odrv3, &odrv4, &odrv5};
ODriveCAN* odrives_can3[] = {&odrv6, &odrv7, &odrv8};

template<typename Func, typename Tuple>
void odriveCommandWrapper(Func&& f, Tuple&& args) {
    std::apply(std::forward<Func>(f), std::forward<Tuple>(args));
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
  FlexCAN_T4_Base* can_ptr_; // pointer to base class of FlexCAN_T4<CAN*,...>
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data(ODRV0_CAN_BUS_ID, ODRV0_CAN_ORDER_ID, &ODRV0_CAN);
ODriveUserData odrv1_user_data(ODRV1_CAN_BUS_ID, ODRV1_CAN_ORDER_ID, &ODRV1_CAN);
ODriveUserData odrv2_user_data(ODRV2_CAN_BUS_ID, ODRV2_CAN_ORDER_ID, &ODRV2_CAN);
ODriveUserData odrv3_user_data(ODRV3_CAN_BUS_ID, ODRV3_CAN_ORDER_ID, &ODRV3_CAN);
ODriveUserData odrv4_user_data(ODRV4_CAN_BUS_ID, ODRV4_CAN_ORDER_ID, &ODRV4_CAN);
ODriveUserData odrv5_user_data(ODRV5_CAN_BUS_ID, ODRV5_CAN_ORDER_ID, &ODRV5_CAN);
ODriveUserData odrv6_user_data(ODRV6_CAN_BUS_ID, ODRV6_CAN_ORDER_ID, &ODRV6_CAN);
ODriveUserData odrv7_user_data(ODRV7_CAN_BUS_ID, ODRV7_CAN_ORDER_ID, &ODRV7_CAN);
ODriveUserData odrv8_user_data(ODRV8_CAN_BUS_ID, ODRV8_CAN_ORDER_ID, &ODRV8_CAN);
ODriveUserData* odrives_data[] = {&odrv0_user_data, &odrv1_user_data, &odrv2_user_data, &odrv3_user_data, &odrv4_user_data, &odrv5_user_data, &odrv6_user_data, &odrv7_user_data, &odrv8_user_data};

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->received_feedback = true;
  sys_data_->setEncoderEstimateAtBusAndNode(msg.Pos_Estimate, msg.Vel_Estimate, odrv_user_data->bus_idx_, odrv_user_data->node_idx_);
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    /*Serial.print("0 |");
    Serial.print(msg.mb);
    Serial.print(" | ID: 0x");
    Serial.println(msg.id, HEX);*/
    onReceive(msg, *odrive);
  }
}

std::unordered_map<uint32_t, uint32_t> can_id_counts;
unsigned long last_report_time = 0;

void onCanMessageLog(const CAN_message_t &msg) {
  can_id_counts[msg.id]++;
  
  // Optional: Print every N milliseconds
  if (millis() - last_report_time > 5000) {
    Serial.println("---- CAN ID Frequencies ----");
    for (const auto& entry : can_id_counts) {
      Serial.printf("ID: 0x%03X, Count: %lu\n", entry.first, entry.second);
    }
    Serial.println("----------------------------");
    last_report_time = millis();
    can_id_counts.clear();  // Reset for next interval
  }
}

void onCanMessage1(const CanMsg& msg) {
  for (auto odrive: odrives_can1) {
    onReceive(msg, *odrive);
  }
}

void onCanMessage2(const CanMsg& msg) {
  for (auto odrive: odrives_can2) {
    onReceive(msg, *odrive);
  }
}

void onCanMessage3(const CanMsg& msg) {
  for (auto odrive: odrives_can3) {
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

    // Intiialize SystemDataContainer taking into account number of ODrives per CAN bus
    sys_data_ = std::make_unique<SystemDataContainer>();
    sys_data_->add(SystemData<N_ODRIVE_CAN1>());
    sys_data_->add(SystemData<N_ODRIVE_CAN2>());
    sys_data_->add(SystemData<N_ODRIVE_CAN3>());

    num_odrives = sizeof(odrives) / sizeof(odrives[0]);
    num_odrives_data = sizeof(odrives_data) / sizeof(odrives_data[0]);
    if (num_odrives != num_odrives_data)
    {
      Serial.println("Error: num_odrives != num_odrives_data");
      while(true);
    }

    // Register callbacks for the heartbeat and encoder feedback messages
    for (size_t i = 0; i < num_odrives; ++i)
    {
      odrives[i]->onFeedback(onFeedback, odrives_data[i]);
      odrives[i]->onStatus(onHeartbeat, odrives_data[i]);
    }

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCAN()) {
      Serial.println("CAN failed to initialize: reset required");
      while (true); // spin indefinitely
    }

    // Disable *_msg_rate_ms
    for (auto odrive: odrives)
    {
      while(!odrive->setEndpoint(276, 0)) {delay(10);} // disable iq_msg_rate_ms
      Serial.print("x");

      while(!odrive->setEndpoint(277, 0)) {delay(10);} // disable error_msg_rate_ms
      Serial.print("x");

      while(!odrive->setEndpoint(278, 0)) {delay(10);} // disable temperature_msg_rate_ms
      Serial.print("x");

      while(!odrive->setEndpoint(279, 0)) {delay(10);} // disable bus_voltage_msg_rate_ms
      Serial.print("x");

      while(!odrive->setEndpoint(280, 0)) {delay(10);} // disable torques_msg_rate_ms
      Serial.println("x");
    }

    for (size_t i = 0; i < num_odrives; ++i)
    {
      for (int j = 0; j < 5; ++j) {
          delay(10);
          pumpEvents(*odrives_data[i]->can_ptr_);
      }
    }

    for (auto odrive: odrives)
    {
      Serial.print(">");
      Serial.print(odrive->getEndpoint<uint32_t>(274)); // heartbeat
      Serial.print(" ");
      Serial.print(odrive->getEndpoint<uint32_t>(275)); // encoder
      Serial.print(" | ");
      Serial.print(odrive->getEndpoint<uint32_t>(276));
      Serial.print(" ");
      Serial.print(odrive->getEndpoint<uint32_t>(277));
      Serial.print(" ");
      Serial.print(odrive->getEndpoint<uint32_t>(278));
      Serial.print(" ");
      Serial.print(odrive->getEndpoint<uint32_t>(279));
      Serial.print(" ");;
      Serial.print(odrive->getEndpoint<uint32_t>(280));
      Serial.println(" ");
    }

    Serial.print("Found ODrives: ");
    for (size_t i = 0; i < num_odrives; ++i)
    {
      while (!odrives_data[i]->received_heartbeat)
      {
        pumpEvents(*odrives_data[i]->can_ptr_);
        delay(1);
      }
      Serial.print(" [odrv");
      Serial.print(i);
      Serial.print("] ");
    }
    Serial.println("");

    // request bus voltage and current (1sec timeout)
    /*Serial.print("Request bus voltage and current:");
    Get_Bus_Voltage_Current_msg_t vbus;
    for (size_t i = 0; i < num_odrives; ++i)
    {
      while(!odrives[i]->request(vbus, 1)) {delay(100);}
      Serial.print(" [odrv");
      Serial.print(i);
      Serial.print("] ");
    }
    Serial.println("");

    // print the last odrive's DC voltage and current
    Serial.print("DC voltage [V]: ");
    Serial.print(vbus.Bus_Voltage);
    Serial.print(" | DC current [A]: ");
    Serial.println(vbus.Bus_Current);*/
  
    // enabling closed loop control
    Serial.print("Enabling closed-loop control: ");
    for (size_t i = 0; i < num_odrives; ++i)
    {
      while (odrives_data[i]->last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[i]->clearErrors();
        delay(1);
        odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        // Pump events for 150ms. This delay is needed for two reasons;
        // 1. If there is an error condition, such as missing DC power, the ODrive might
        //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
        //    on the first heartbeat response, so we want to receive at least two
        //    heartbeats (100ms default interval).
        // 2. If the bus is congested, the setState command won't get through
        //    immediately but can be delayed.
        for (int j = 0; j < 15; ++j) {
          delay(10);
          pumpEvents(*odrives_data[i]->can_ptr_);
        }
      }
      Serial.print(" [odrv");
      Serial.print(i);
      Serial.print("] ");
    }
    Serial.println("");
    
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
    /*can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
    can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    //can1.setMBFilter(MB1, 0, 0x0);
    //can1.enhanceFilter(MB1);
    can1.enableMBInterrupts(); // Tells the hardware to trigger interrupt when a message is received or sent on any mailbox. Required to call onReceive();
    can1.onReceive(onCanMessage1);
    //can1.onReceive(onCanMessageLog);
    //can1.mailboxStatus();
    can1.distribute();
    can1.setClock(CLK_60MHz);
    can1.mailboxStatus();*/

    can1.begin();
    can1.setMaxMB(20);  // Slight buffer for TX or expansion

    // High-frequency IDs (assign 2 mailboxes each)
    uint16_t highFreqIDs[] = { 0x009, 0x029, 0x049 };
    int mb = 0;
    for (int i = 0; i < 3; i++) {
      can1.setMB(mb, RX);
      can1.setMBFilter(mb, highFreqIDs[i]);
      //can1.setMBMask(mb, 0x7FF);
      mb++;

      can1.setMB(mb, RX);
      can1.setMBFilter(mb, highFreqIDs[i]);
      //can1.setMBMask(mb, 0x7FF);
      mb++;
    }

    // Lower-frequency IDs (1 mailbox each)
    uint16_t lowFreqIDs[] = { 0x001, 0x021, 0x041, 0x005, 0x025, 0x045 };
    for (int i = 0; i < 6; i++) {
      can1.setMB(mb, RX);
      can1.setMBFilter(mb, lowFreqIDs[i]);
      //can1.setMBMask(mb, 0x7FF);
      mb++;
    }

    // Fallback/wildcard mailboxes
    for (int i = 0; i < 3; i++) {
      can1.setMB(mb, RX);         // Standard
      can1.setMBFilter(mb, 0x000); // Match any ID
      //can1.setMBMask(mb, 0x000);   // Wildcard
      mb++;
    }

    for (int i = 0; i < 5; i++) {
      can1.setMB(mb, TX);
      mb++;
    }
    // Setup receive interrupt
    can1.enableMBInterrupts();
    can1.onReceive(onCanMessage1);
    can1.setClock(CLK_60MHz);
    can1.mailboxStatus();


    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);
    can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    //can2.setMBFilter(MB1, 0, 0x0);
    //can2.enhanceFilter(MB1);
    can2.enableMBInterrupts();
    can2.onReceive(onCanMessage2); // TODO(@nicholasadr): should use a separate onCanMessage?

    can2.distribute();
    can2.setClock(CLK_60MHz);
    //can2.mailboxStatus();

    can3.begin();
    can3.setBaudRate(CAN_BAUDRATE);
    can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    //can3.setMBFilter(MB1, 0, 0x0);
    //can3.enhanceFilter(MB1);
    can3.enableMBInterrupts();
    can3.onReceive(onCanMessage3); // TODO(@nicholasadr): should use a separate onCanMessage?

    can3.distribute();
    can3.setClock(CLK_60MHz);
    // can3.mailboxStatus();

    return true;
}


void loop()
{
     // This is required on some platforms to handle incoming feedback CAN messages
    pumpEvents(can1);
    pumpEvents(can2);
    pumpEvents(can3);
    //can1.mailboxStatus();

    parseAndProcessUDPPacket(); // receive UDP message from UP to Teensy
    if (!first_packet_recv)
        return;

    //unsigned long start_time_mcs = micros();
    sendUDPPacket(); // send UDP message from Teensy to UP
    //CAN_udp_msg_send_time_mcs = micros() - start_time_mcs;

    /*unsigned long current_time_mcs = micros();
    unsigned long loop_duration_mcs = current_time_mcs - prev_time_mcs;
    prev_time_mcs = current_time_mcs;
    Serial.print("[CAN loop timing] ");
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
    //unsigned long start_time_mcs = micros();
    int size = udp.parsePacket();
    //CAN_udp_msg_parse_time_mcs = micros() - start_time_mcs;

    //start_time_mcs = micros();
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
              for (size_t i = 0; i < num_odrives; ++i)
              {
                odriveCommandWrapper([&](Input_Pos_TYPE p, Vel_FF_TYPE v_ff, Torque_FF_TYPE t_ff) { odrives[i]->setPosition(p, v_ff, t_ff); },
                    cmd.getCommandValue());
              }
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
              for (size_t i = 0; i < num_odrives; ++i)
              {
                odriveCommandWrapper([&](Input_Vel_TYPE v, Input_Torque_FF_TYPE t_ff) { odrives[i]->setVelocity(v, t_ff); },
                    cmd.getCommandValue());
              }
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
              for (size_t i = 0; i < num_odrives; ++i)
              {
                odriveCommandWrapper([&](Input_Torque_TYPE t) { odrives[i]->setTorque(t); },
                      cmd.getCommandValue());
              }
            }
            break;
        }
        default: {
            PRINTLN("Unknown MsgType!");
            break;
        }
        }
    }
    //CAN_udp_msg_process_time_mcs = micros() - start_time_mcs;
}

bool receivedFeedbackOnAllODrives()
{
  bool received_all = true;

  for (size_t i = 0; i < num_odrives; ++i)
  {
    received_all = received_all && odrives_data[i]->received_feedback;  
  }

  return received_all;
}

bool resetODriveData()
{
  for (size_t i = 0; i < num_odrives; ++i)
  {
    odrives_data[i]->received_feedback = false;
  }
}

void sendUDPPacket()
{
    if (receivedFeedbackOnAllODrives()) {
      //Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      //Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      odrv0_user_data.received_feedback = false;
      odrv1_user_data.received_feedback = false;
      odrv2_user_data.received_feedback = false;

      //SystemData packet(feedback.Pos_Estimate, feedback.Vel_Estimate);
      uint8_t buffer[sys_data_->dataSize()];
      //packet.serialize(buffer);
      sys_data_->serialize(buffer);

      if (!udp.send("10.176.32.14", PC_udp_port_listening, buffer, sizeof(buffer)))
      {
          printf("Error sending udp from Teensy\n");
      }
    }
}
