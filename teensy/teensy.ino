#include <iostream>
#include <cstring>
#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include <QNEthernet.h>
#include "Command.h"

#define CAN_BAUDRATE 250000 // CAN Simple can go up to 1e6?
#define ODRV0_NODE_ID 0

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

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

//std::unique_ptr<CommandBase> CommandBase::fromBuffer(const std::vector<uint8_t>& buffer)
std::unique_ptr<CommandBase> fromBuffer(const std::vector<uint8_t>& buffer)
{
    if (buffer.empty()) {
        std::cerr << "Empty buffer!\n";
        return nullptr;
    }

    MsgType type = static_cast<MsgType>(buffer[0]);
    // TODO(@nicholasadr): unnecessary?
    std::vector<uint8_t> payload(buffer.begin() + 1, buffer.end());

    switch (type) {
        case MsgType::PositionCommand:
        {
            Serial.println("MsgType::PositionCommand");
            PositionCommand cmd;
            cmd.deserialize(payload);
            return std::make_unique<PositionCommand>(cmd);
        }
        case MsgType::TorqueCommand:
        {
            Serial.println("MsgType::TorqueCommand");
            TorqueCommand cmd;
            cmd.deserialize(payload);
            return std::make_unique<TorqueCommand>(cmd);
        }
        default:
            Serial.println("Unknown MsgType");
            std::cerr << "Unknown MsgType!\n";
            return nullptr;
    }
}

struct SystemData {
    float encoder_Pos_Estimate;  // [rev]
    float encoder_Vel_Estimate;  // [rev/s]

    // Constructor to initialize default values
    SystemData() : encoder_Pos_Estimate(0.0f), encoder_Vel_Estimate(0.0f) {}

    // Constructor from input arguments
    SystemData(float pos_estimate, float vel_estimate) : encoder_Pos_Estimate(pos_estimate), encoder_Vel_Estimate(vel_estimate) {}

    // Serialize the structure into a byte array
    void serialize(uint8_t* buffer) const {
        // Serialize Pos_Estimate and Vel_Estimate into the buffer
        std::memcpy(buffer, &encoder_Pos_Estimate, sizeof(encoder_Pos_Estimate));
        std::memcpy(buffer + sizeof(encoder_Pos_Estimate), &encoder_Vel_Estimate, sizeof(encoder_Vel_Estimate));
    }

    // Deserialize from byte array
    static SystemData deserialize(const std::vector<uint8_t>& buffer) {
        SystemData msg;

        // Deserialize Pos_Estimate and Vel_Estimate from the buffer
        std::memcpy(&msg.encoder_Pos_Estimate, buffer.data(), sizeof(encoder_Pos_Estimate));
        std::memcpy(&msg.encoder_Vel_Estimate, buffer.data() + sizeof(encoder_Pos_Estimate), sizeof(encoder_Vel_Estimate));

        return msg;
    }
};

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

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can2), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
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

    // Register callbacks for the heartbeat and encoder feedback messages
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCAN()) {
      Serial.println("CAN failed to initialize: reset required");
      while (true); // spin indefinitely
    }

    Serial.println("Waiting for ODrive...");
    while (!odrv0_user_data.received_heartbeat) {
      pumpEvents(can2);
      delay(100);
    }

    Serial.println("found ODrive");

    // request bus voltage and current (1sec timeout)
    Serial.println("attempting to read bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus;
    if (!odrv0.request(vbus, 1)) {
      Serial.println("vbus request failed!");
      while (true); // spin indefinitely
    }

    Serial.print("DC voltage [V]: ");
    Serial.println(vbus.Bus_Voltage);
    Serial.print("DC current [A]: ");
    Serial.println(vbus.Bus_Current);
  
    Serial.println("Enabling closed loop control...");
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
        pumpEvents(can2);
      }
    }
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
    return true;
}


void loop()
{
    pumpEvents(can2); // This is required on some platforms to handle incoming feedback CAN messages

    receiveUDPPacket(); // receive UDP message from UP to Teensy
    if (!first_packet_recv)
        return;

    sendCANCommandToODrive(); // send CAN command from Teensy to ODrive Pro
    sendUDPPacket(); // send UDP message from Teensy to UP
}

void receiveUDPPacket()
{
    int size = udp.parsePacket();

    if (size >= 0)
    {
        Serial.println("Received UDP packet");
        const uint8_t *data = udp.data();
        
        if (!first_packet_recv)
            first_packet_recv = true;

        // Extract the payload data
        std::vector<uint8_t> payload(data, data+sizeof(float)*2);

        // Calculate CRC-8 for the payload
        uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());

        if (size == sizeof(float)*2+1)
            {
                const uint8_t received_crc = data[sizeof(float)*2];

                if (received_crc == calculated_crc)
                {
                    // CRC check passed, process the data
                    Serial.println("crc ok");
                    sys_command_ = fromBuffer(payload);
                }
                else
                {
                    // CRC check failed, discard the data
                    printf("CRC check failed\n");
                    printf("Received CRC: %02X\n", received_crc);
                    printf("Calculated CRC: %02X\n", calculated_crc);
                }
            }
            else
            {
                printf("Invalid packet size\n");
            }
    }
}

void sendCANCommandToODrive()
{
    if (sys_command_)
    {   
        //printf("sys_command_ position: %.2f\n", sys_command_.position);
        //printf("sys_command_ vel_ff  : %.2f\n", sys_command_.velocity_ff);
        sys_command_->printValue();
        //odrv0.setPosition(sys_command_->position, sys_command_->velocity_ff);
    }
}

void sendUDPPacket()
{
    if (odrv0_user_data.received_feedback) {
      Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      odrv0_user_data.received_feedback = false;

      SystemData packet(feedback.Pos_Estimate, feedback.Vel_Estimate);
      uint8_t buffer[sizeof(float) * 2];
      packet.serialize(buffer);

      if (!udp.send("10.176.32.14", PC_udp_port_listening, buffer, sizeof(buffer)))
      {
          printf("Error sending udp from Teensy\n");
      }
    } 
}
