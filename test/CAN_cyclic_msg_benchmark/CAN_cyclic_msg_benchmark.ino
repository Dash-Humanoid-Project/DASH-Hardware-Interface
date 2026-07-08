/* Analyse rate of incoming cyclic messages from multiple ODrives. */

#define TEENSY_4_1

// Interval between msg in ms. Set to 0 to disable
#define HEARTBEAT_MSG_RATE_MS 100
#define ENCODER_MSG_RATE_MS  1    // 500 Hz
#define MAX_ODRIVE_COUNT 18
#define ENDPOINT_WRITE_RETRIES 5

// How often the stats block prints, in ms.
#define STATS_PRINT_INTERVAL_MS 1000

#include <FlexCAN_T4.h>
#undef CAN_ERROR_BUS_OFF // TODO: macro name conflict in FlexCAN_T4/imxrt_flexcan.h and ODriveEnums.h
#include "ODriveCAN.h"
#include "ODriveFlexCAN.hpp"
#include "Param.h"

#define CAN_BAUDRATE 250000 // CAN Simple can go up to 1e6?

#define ODRIVE_CAN_CMD_MASK 0x1F // ODriveCAN::kCmdIdBits
#define ODRIVE_CAN_NODE_SHIFT 5  // ODriveCAN::kNodeIdShift

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

// Bench setup (2-3-0): CAN1 carries nodes 0-1, CAN2 carries nodes 2-4 (node 4
// / l_ankle was moved off CAN3 onto CAN2), CAN3 is unused. Nodes 5-8 (right
// leg, Teensy 2) are not wired to this Teensy and are intentionally left out.
ODriveCAN odrv0(wrap_can_intf(ODRV0_CAN), ODRV0_CAN_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(ODRV1_CAN), ODRV1_CAN_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(ODRV2_CAN), ODRV2_CAN_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(ODRV3_CAN), ODRV3_CAN_NODE_ID);
ODriveCAN odrv4(wrap_can_intf(ODRV4_CAN), ODRV4_CAN_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3, &odrv4}; // ordered by node id 0..4

size_t num_odrives = 0;

// packet counters
volatile uint32_t heartbeat_count[MAX_ODRIVE_COUNT] = {0};
volatile uint32_t encoder_count[MAX_ODRIVE_COUNT] = {0};
// previous snapshot for delta calculation
uint32_t prev_heartbeat[MAX_ODRIVE_COUNT] = {0};
uint32_t prev_encoder[MAX_ODRIVE_COUNT] = {0};

// Inter-arrival gap tracking for encoder messages — distinguishes many small,
// evenly-spaced skipped ticks (points at the ODrive's own TX scheduling)
// from a few large bursty stalls (points at something on the Teensy side).
#define EXPECTED_ENCODER_GAP_US (ENCODER_MSG_RATE_MS * 1000UL)
#define SKIP_GAP_THRESHOLD_US (EXPECTED_ENCODER_GAP_US + EXPECTED_ENCODER_GAP_US / 2) // 1.5x nominal
volatile uint32_t last_encoder_us[MAX_ODRIVE_COUNT] = {0};
volatile uint32_t max_encoder_gap_us[MAX_ODRIVE_COUNT] = {0};
volatile uint32_t encoder_skip_count[MAX_ODRIVE_COUNT] = {0};

elapsedMillis print_timer;

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  uint16_t node_id = msg.id >> ODRIVE_CAN_NODE_SHIFT;
  uint16_t cmd_id = msg.id & ODRIVE_CAN_CMD_MASK;

  if (node_id >= MAX_ODRIVE_COUNT) return;

  switch (cmd_id) {
    case 0x001:
      heartbeat_count[node_id]++;
      break;
    case 0x009: {
      encoder_count[node_id]++;
      uint32_t now = micros();
      uint32_t last = last_encoder_us[node_id];
      last_encoder_us[node_id] = now;
      if (last != 0) {
        uint32_t gap = now - last; // wraps safely (unsigned)
        if (gap > max_encoder_gap_us[node_id]) max_encoder_gap_us[node_id] = gap;
        if (gap > SKIP_GAP_THRESHOLD_US) encoder_skip_count[node_id]++;
      }
      break;
    }
    default:
      break; // ignore other messages
  }

  // Forward to the matching ODriveCAN object so getEndpoint()/awaitMsg() can
  // actually see request-response traffic (e.g. TxSdo replies). Without this,
  // getEndpoint() always times out and returns 0 regardless of whether the
  // corresponding write actually landed.
  if (node_id < num_odrives) {
    onReceive(msg, *odrives[node_id]);
  }
}

// odrive->setEndpoint() is fire-and-forget — it always returns true regardless
// of whether the ODrive actually received/applied the write (see ODriveCAN.h).
// So the only way to know a config write landed is to read it back.
template <typename T>
bool writeAndVerifyEndpoint(ODriveCAN* odrive, uint16_t node_id, uint16_t endpoint_id,
                             T value, const char* name) {
  for (int attempt = 0; attempt < ENDPOINT_WRITE_RETRIES; ++attempt) {
    odrive->setEndpoint(endpoint_id, value);
    delay(10);
    T readback = odrive->getEndpoint<T>(endpoint_id);
    if (readback == value) {
      if (attempt > 0) {
        Serial.printf("Node %2d: %s confirmed at %ld after %d attempt(s)\n",
                      node_id, name, (long)value, attempt + 1);
      }
      return true;
    }
  }
  Serial.printf("Node %2d: FAILED to set %s (wanted %ld, last read-back %ld)\n",
                node_id, name, (long)value, (long)odrive->getEndpoint<T>(endpoint_id));
  return false;
}

bool setupCAN();

// ODriveCAN::onReceive() prints "missing callback" (blocking Serial.println,
// called synchronously from inside the CAN ISR via our onCanMessage ->
// onReceive forwarding) for every Heartbeat message if no onStatus callback
// is registered. We don't need the decoded status — we're counting raw
// messages ourselves — but we do need *some* callback registered so the
// library doesn't spam a blocking print from interrupt context 10x/sec/node.
void noopHeartbeatCallback(Heartbeat_msg_t&, void*) {}

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

    // Register before setupCAN() enables interrupts, so there's no window
    // where a Heartbeat could arrive with no callback registered yet.
    for (size_t i = 0; i < num_odrives; ++i) {
      odrives[i]->onStatus(noopHeartbeatCallback);
    }

    // Configure and initialize the CAN bus interface. This function depends on
    // your hardware and the CAN stack that you're using.
    if (!setupCAN()) {
      Serial.println("CAN failed to initialize: reset required");
      while (true); // spin indefinitely
    }

    Serial.println("Configuring ODrive message rates...");
    for (size_t i = 0; i < num_odrives; ++i) {
      uint16_t node_id = (uint16_t)i; // odrives[] is ordered by node id 0..4
      writeAndVerifyEndpoint(odrives[i], node_id, 274, HEARTBEAT_MSG_RATE_MS, "heartbeat_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 275, ENCODER_MSG_RATE_MS,  "encoder_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 276, 0, "iq_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 277, 0, "error_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 278, 0, "temperature_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 279, 0, "bus_voltage_msg_rate_ms");
      writeAndVerifyEndpoint(odrives[i], node_id, 280, 0, "torques_msg_rate_ms");
    }
    Serial.println("Done configuring message rates.");

    for (int j = 0; j < 5; ++j) {
      delay(10);
      pumpEvents(can1);
      pumpEvents(can2);
    }
}

// Configures explicit per-ID RX mailboxes on `bus` so each high/low frequency
// CAN ID gets a dedicated mailbox, instead of relying on FlexCAN_T4's
// .distribute()/hardware defaults. At 100 Hz that fallback looked fine, but at
// the real 500 Hz production rate it silently dropped ~80% of one node's
// traffic on CAN2 while its neighbor stayed clean — this uniform, explicit
// setup replaces that with a filter layout matching the one already proven
// reliable on CAN1.
template <typename CANBus>
void configureFilteredBus(CANBus &bus, const uint16_t* highFreqIDs, size_t nHigh,
                           const uint16_t* lowFreqIDs, size_t nLow,
                           uint8_t nWildcard, uint8_t nTx, uint8_t mbPerHighID = 2) {
  int mb = 0;
  for (size_t i = 0; i < nHigh; i++) {
    for (uint8_t d = 0; d < mbPerHighID; d++) {
      bus.setMB(mb, RX); bus.setMBFilter(mb, highFreqIDs[i]); mb++;
    }
  }
  for (size_t i = 0; i < nLow; i++) {
    bus.setMB(mb, RX); bus.setMBFilter(mb, lowFreqIDs[i]); mb++;
  }
  for (int i = 0; i < nWildcard; i++) {
    bus.setMB(mb, RX);
    bus.setMBFilter(mb, ACCEPT_ALL); // catches anything not in the lists above
    mb++;
  }
  for (int i = 0; i < nTx; i++) { bus.setMB(mb, TX); mb++; }

  bus.setMaxMB(mb);
  bus.enableMBInterrupts();
  bus.onReceive(onCanMessage);
  bus.setClock(CLK_60MHz);
}

bool setupCAN()
{
    can1.begin();
    can1.setBaudRate(CAN_BAUDRATE);
    can2.begin();
    can2.setBaudRate(CAN_BAUDRATE);

    // CAN1: nodes 0,1
    uint16_t can1High[] = { 0x009, 0x029 };
    uint16_t can1Low[]  = { 0x001, 0x021 };
    configureFilteredBus(can1, can1High, 2, can1Low, 2, /*wildcard*/2, /*tx*/5);

    // CAN2: nodes 2,3,4 (l_ankle moved here from CAN3 — 2-3-0 bench layout)
    uint16_t can2High[] = { 0x049, 0x069, 0x089 };
    uint16_t can2Low[]  = { 0x041, 0x061, 0x081 };
    configureFilteredBus(can2, can2High, 3, can2Low, 3, 2, 5);

    return true;
}

void loop()
{
     // This is required on some platforms to handle incoming feedback CAN messages
    pumpEvents(can1);
    pumpEvents(can2);

    if (print_timer >= STATS_PRINT_INTERVAL_MS)
    {
      Serial.printf("=== Packet Stats (per %lu ms) ===\n", (unsigned long)STATS_PRINT_INTERVAL_MS);
      for (int node = 0; node < num_odrives; node++) {
        uint32_t hb = heartbeat_count[node];
        uint32_t enc = encoder_count[node];
        uint32_t dhb = hb - prev_heartbeat[node];
        uint32_t denc = enc - prev_encoder[node];
        if (dhb > 0 || denc > 0) {
          Serial.printf("Node %2d: Heartbeats: %5lu/%-5lu  Encoders: %6lu/%-6lu  MaxGap: %5lu us  Skips(>%lu us): %lu\n",
                        node, dhb,
                        (unsigned long)(STATS_PRINT_INTERVAL_MS/HEARTBEAT_MSG_RATE_MS), denc,
                        (unsigned long)(STATS_PRINT_INTERVAL_MS/ENCODER_MSG_RATE_MS),
                        (unsigned long)max_encoder_gap_us[node],
                        (unsigned long)SKIP_GAP_THRESHOLD_US,
                        (unsigned long)encoder_skip_count[node]);
        }

        max_encoder_gap_us[node] = 0;
        encoder_skip_count[node] = 0;
        prev_heartbeat[node] = hb;
        prev_encoder[node] = enc;
      }
      Serial.println();
      print_timer = 0;
    }
}
