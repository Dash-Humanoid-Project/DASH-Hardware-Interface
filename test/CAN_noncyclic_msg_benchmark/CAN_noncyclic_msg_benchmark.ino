/* Analyse packet loss and latency of noncyclic messages from multiple ODrives. */

#define TEENSY_4_1

#define TIMEOUT_MS 10 // how long to wait for endpoint to return message
#define MAX_ODRIVE_COUNT 18

#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include "Param.h"

#define CAN_BAUDRATE 250000 // CAN Simple can go up to 1e6?

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// Instantiate ODrive objects early for fromBuffer()
ODriveCAN odrv0(wrap_can_intf(ODRV0_CAN), ODRV0_CAN_NODE_ID);
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

size_t num_odrives = 0;

uint32_t success_count[MAX_ODRIVE_COUNT] = {0};
uint32_t failure_count[MAX_ODRIVE_COUNT] = {0};
double total_latency_ms[MAX_ODRIVE_COUNT] = {0.0};

elapsedMillis print_timer;

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

    num_odrives = sizeof(odrives) / sizeof(odrives[0]);

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCAN()) {
      Serial.println("CAN failed to initialize: reset required");
      while (true); // spin indefinitely
    }

    // Disable *_msg_rate_ms
    for (auto odrive: odrives)
    {
      while(!odrive->setEndpoint(274, 0)) {delay(10);} // disable heartbeat
      while(!odrive->setEndpoint(275, 0)) {delay(10);} // disable encoder
      while(!odrive->setEndpoint(276, 0)) {delay(10);} // disable iq_msg_rate_ms
      while(!odrive->setEndpoint(277, 0)) {delay(10);} // disable error_msg_rate_ms
      while(!odrive->setEndpoint(278, 0)) {delay(10);} // disable temperature_msg_rate_ms
      while(!odrive->setEndpoint(279, 0)) {delay(10);} // disable bus_voltage_msg_rate_ms
      while(!odrive->setEndpoint(280, 0)) {delay(10);} // disable torques_msg_rate_ms
    }

    for (int j = 0; j < 5; ++j) {
      delay(10);
      pumpEvents(can1);
      pumpEvents(can2);
      pumpEvents(can3);
    }
}

bool setupCAN()
{
    can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
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
      mb++;
    }

    // Lower-frequency IDs (1 mailbox each)
    uint16_t lowFreqIDs[] = { 0x001, 0x021, 0x041, 0x005, 0x025, 0x045 };
    for (int i = 0; i < 6; i++) {
      can1.setMB(mb, RX);
      can1.setMBFilter(mb, lowFreqIDs[i]);
      mb++;
    }

    // Fallback/wildcard mailboxes
    for (int i = 0; i < 3; i++) {
      can1.setMB(mb, RX);         // standard
      can1.setMBFilter(mb, 0x000); // match any ID
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
    //can1.mailboxStatus(); // for checking individual mailbox status

    // TODO: the startup doesn't work if I set CAN2 and CAN3 with manual mailbox assignment similar to CAN1
    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);
    can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can2.enableMBInterrupts();
    can2.onReceive(onCanMessage2);
    can2.distribute();
    can2.setClock(CLK_60MHz);
    //can2.mailboxStatus();

    can3.begin();
    can3.setBaudRate(CAN_BAUDRATE);
    can3.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    can3.enableMBInterrupts();
    can3.onReceive(onCanMessage3);
    can3.distribute();
    can3.setClock(CLK_60MHz);
    //can3.mailboxStatus();
    
    return true;
}

void loop()
{
     // This is required on some platforms to handle incoming feedback CAN messages
    pumpEvents(can1);
    pumpEvents(can2);
    pumpEvents(can3);

    for (int i=0; i < num_odrives; i++)
    {
      ODriveCAN* odrive = odrives[i];

      uint32_t start_us = micros();
      double success = odrive->getEndpoint<uint32_t>(225, TIMEOUT_MS); // request for encoder pos_estimate

      if (success) {
        uint32_t elapsed_us = micros() - start_us;
        total_latency_ms[i] += elapsed_us / 1000.0;  // convert to ms
        success_count[i]++;
      } else {
        failure_count[i]++;
      }
    }
    
    if (print_timer >= 5000) // analyse every 5 second
    {
      Serial.println("=== ODrive Request Stats ===");
      for (int i = 0; i < num_odrives; i++) {
        double avg_latency = (success_count[i] > 0) ? (total_latency_ms[i] / success_count[i]) : 0.0;
        Serial.printf("Node %2d: Avg Time: %.2f ms | Success: %lu | Lost: %lu\n",
                      i, avg_latency, success_count[i], failure_count[i]);

        success_count[i] = 0;
        failure_count[i] = 0;
        total_latency_ms[i] = 0.0;
      }
      Serial.println();
      print_timer = 0;
    }
}
