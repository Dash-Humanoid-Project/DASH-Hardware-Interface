// One-time absolute-encoder ("zero it here, never home again") setup for one
// joint, entirely over CAN — no USB required.
//
// The naive approach (write pos_vel_mapper.config.offset = the current raw
// absolute reading, so raw - offset = 0) does NOT reliably produce
// pos_estimate = 0 after the save_configuration()-triggered reboot: extensive
// live testing against the right hip_roll board (2026-09-10, including two
// real physical power-cycles, not just soft reboots) showed the actual
// post-reboot result is offset by a fixed-but-unexplained per-board constant
// we could not derive from any exposed/documented field (ODrive's 0.6.x
// pos_vel_mapper firmware source is NDA-only, so the exact algorithm isn't
// inspectable). The relationship IS deterministic and reproducible for a
// given board/state, though: one calibration attempt tells you exactly how
// far off the next one needs to correct by. Hence the two-shot approach below
// — it's board-agnostic and doesn't need to know that constant in advance.
//
// 2026-10-22: pos_vel_mapper.working_offset (see EP_WORKING_OFFSET below)
// tracks closely with this constant and is NOT merely diagnostic as its own
// comment claimed — but a same-day attempt to derive an exact formula for it
// (pos_estimate == (raw - offset) + working_offset, assuming pos_rel == raw -
// offset) led to flipping this section's correction to "offset + error",
// which a live two-pass test then DISPROVED: offset moved down by exactly
// the first pass's error, and the second pass's error came back almost
// exactly DOUBLED, not cancelled — and a later readback showed pos_rel did
// NOT match raw - offset at all (pos_rel stayed near 0 while raw - offset
// was ~0.176), invalidating that derivation's key assumption. pos_rel
// appears to instead track movement since the axis last armed/calibrated,
// not a function of raw/offset directly.
//
// Reverted to the original "offset - error" below on the strength of that
// direct empirical test (increasing offset moved pos_estimate the SAME
// direction, so cancelling it means moving offset the OTHER way) rather
// than a model of the internal mechanism, which remains unconfirmed —
// ODrive's 0.6.x pos_vel_mapper firmware source is NDA-only, so there's no
// way to derive this from first principles, only from what hardware
// testing actually shows. Treat any future claim about the exact internal
// formula here with real skepticism until it's survived a live two-pass
// test like this one, not just matched one reading by coincidence.
//
//   1st pass: offset = raw (freshly read over CAN), offset_valid=true,
//             approx_init_pos=0.0, approx_init_pos_valid=true,
//             absolute_setpoints=true, save_configuration() (reboots)
//   read back the resulting axis0.pos_estimate ("error")
//   2nd pass: offset = (1st pass's offset) - error, save_configuration()
//   verify pos_estimate is now within tolerance of 0
//
// "Zero it here" means exactly that: wherever the joint physically is RIGHT
// NOW when you run this becomes its permanent zero reference on every future
// boot — confirmed to survive a real power cycle. Don't run this until the
// joint is physically where you want zero to be.
//
// GetSetParam is now wired on all four Teensys (2026-09-21), so this works
// for any leg/arm joint, not just the right leg. Endpoint IDs are
// firmware-build-specific but were confirmed shared across every board
// checked (fw 0.6.11 / hw 4.4.58).
#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include "HardwareBridge.h"
#include "TestUtils.h"

namespace {

constexpr uint16_t EP_OFFSET = 447;
constexpr uint16_t EP_OFFSET_VALID = 446;
constexpr uint16_t EP_APPROX_INIT_POS = 449;
constexpr uint16_t EP_APPROX_INIT_POS_VALID = 448;
constexpr uint16_t EP_ABSOLUTE_SETPOINTS = 390;
constexpr uint16_t EP_SAVE_CONFIGURATION = 710;
constexpr uint16_t EP_RAW_ABSOLUTE = 612;    // rs485_encoder_group0.raw, float, [0,1) turns
constexpr uint16_t EP_POS_ESTIMATE = 225;    // axis0.pos_estimate, float, turns
constexpr uint16_t EP_WORKING_OFFSET = 441;  // pos_vel_mapper.working_offset, float, turns — tracks closely
                                              // with the per-board correction constant this file's header
                                              // discusses, despite its own "diagnostic only" framing, but
                                              // its exact relationship to pos_estimate is NOT confirmed —
                                              // see the 2026-10-22 note in the file header before trusting
                                              // any claimed formula involving it.
constexpr uint16_t EP_POS_REL = 438;         // pos_vel_mapper.pos_rel, float, turns (diagnostic only)

// A post-calibration pos_estimate within this many turns of 0 counts as
// zeroed. 0.01 turns is far above CAN/encoder read noise (~0.0002 turns seen
// during testing) but far below "actually still needs correcting" territory.
constexpr float kZeroToleranceTurns = 0.01f;

// CAN periodic-broadcast rate endpoints (ms; 0 = disabled). teensy2.ino's
// setup() writes these once at *Teensy* boot (see its setEndpoint(274/275/...)
// calls) to enable Heartbeat + Get_Encoder_Estimates broadcast at the rates
// the rest of this project depends on for live feedback. Rebooting only the
// ODrive (via save_configuration(), or a real physical power-cycle) resets
// them to firmware defaults without the Teensy re-running setup(). Harmless
// to re-apply unconditionally (fire-and-forget, matches what the Teensy
// itself already does at its own boot).
constexpr uint16_t EP_HEARTBEAT_RATE_MS = 274;
constexpr uint16_t EP_ENCODER_RATE_MS = 275;
constexpr uint16_t EP_IQ_RATE_MS = 276;
constexpr uint16_t EP_ERROR_RATE_MS = 277;
constexpr uint16_t EP_TEMPERATURE_RATE_MS = 278;
constexpr uint16_t EP_BUS_VOLTAGE_RATE_MS = 279;
constexpr uint16_t EP_TORQUES_RATE_MS = 280;
constexpr int32_t HEARTBEAT_MSG_RATE_MS = 100;
constexpr int32_t ENCODER_MSG_RATE_MS = 2;

void restoreCanBroadcastRates(Leg& leg, const std::string& joint) {
    leg.setParamInt32(joint, EP_HEARTBEAT_RATE_MS, HEARTBEAT_MSG_RATE_MS);
    leg.setParamInt32(joint, EP_ENCODER_RATE_MS, ENCODER_MSG_RATE_MS);
    leg.setParamInt32(joint, EP_IQ_RATE_MS, 0);
    leg.setParamInt32(joint, EP_ERROR_RATE_MS, 0);
    leg.setParamInt32(joint, EP_TEMPERATURE_RATE_MS, 0);
    leg.setParamInt32(joint, EP_BUS_VOLTAGE_RATE_MS, 0);
    leg.setParamInt32(joint, EP_TORQUES_RATE_MS, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // let a few Get_Encoder_Estimates frames land
}

void writeAndVerifyFloat(Leg& leg, const std::string& joint, uint16_t ep, float value, const char* name) {
    leg.setParamFloat(joint, ep, value);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    float readback = leg.getParamFloat(joint, ep);
    std::cout << "  " << name << " set to " << value << ", read back " << readback;
    if (readback != value) {
        std::cout << "  MISMATCH — stopping here, do not proceed." << std::endl;
        throw std::runtime_error(std::string(name) + " did not take the written value");
    }
    std::cout << "  OK" << std::endl;
}

void writeAndVerifyBool(Leg& leg, const std::string& joint, uint16_t ep, bool value, const char* name) {
    leg.setParamBool(joint, ep, value);
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // generous, ruling out a timing race
    bool readback = leg.getParamBool(joint, ep);
    std::cout << "  " << name << " set to " << (value ? "true" : "false")
               << ", read back " << (readback ? "true" : "false");
    if (readback != value) {
        std::cout << "  MISMATCH — stopping here, do not proceed." << std::endl;
        throw std::runtime_error(std::string(name) + " did not take the written value");
    }
    std::cout << "  OK" << std::endl;
}

void saveAndReboot(Leg& leg, const std::string& joint) {
    std::cout << "  Calling save_configuration() (endpoint " << EP_SAVE_CONFIGURATION
              << ") via a WRITE, not a read — that's how you invoke a function endpoint, "
              << "not how you fetch a property. This reboots the ODrive..." << std::endl;
    leg.setParamBool(joint, EP_SAVE_CONFIGURATION, true); // value ignored (0 real inputs); the write is the trigger
    std::cout << "  Waiting 3s for the reboot..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    restoreCanBroadcastRates(leg, joint);
}

// A single post-reboot pos_estimate read can falsely come back as exactly
// 0.0 if the CAN GET races the ODrive still re-establishing comms right
// after save_configuration()'s reboot — ODriveCAN::getEndpoint<T>() returns
// T{} (zero) on a genuine timeout with no way to tell that apart from a
// real value of zero (see ParamResponse's comment in Command.h). A real
// pos_estimate is a continuous physical reading, so landing on EXACTLY 0.0
// is itself the suspicious case, not something to take at face value.
// Confirmed 2026-10-22 on l_hip_yaw: a single post-reboot read reported
// exactly "0 turns" — silently skipped the second corrective pass as a
// false PASS — while the true settled value moments later (in
// printReadback()'s independent re-query of the same endpoint) was
// -0.310786 turns.
//
// Mitigation: take reads a short delay apart and require two consecutive
// ones to agree (within noise) before trusting the result — a real
// timeout-zero won't keep repeating identically once the ODrive is
// actually back and responding with its real reading. Exactly 0.0 agreeing
// twice still isn't trusted outright, since that's exactly what a
// persistent/longer comms gap would also look like.
//
// Throws rather than returning a shrugged-off guess if nothing trustworthy
// ever arrives: confirmed 2026-10-22 that a board can take meaningfully
// longer than saveAndReboot()'s fixed 3s wait to actually recover — silently
// returning 0 in that case let the caller log a false PASS, which is worse
// than failing loudly here. The joint must be held in exactly the zero
// position for this whole function's duration (see this file's "hold the
// joint" messaging in main()) — since this throws instead of looping
// forever, that hold time stays bounded even when the board doesn't
// recover in time, instead of the caller having no idea how much longer to
// keep holding for a result that turns out worthless anyway.
//
// Bounded by wall-clock time, not a fixed attempt count: different boards
// have taken very different amounts of real time to recover in testing
// (l_hip_yaw settled after one extra 200ms read; r_hip_roll repeatedly
// needed much longer than a ~1s retry budget, on top of the initial 3s
// wait) — a time budget adapts to that directly, and generously, rather
// than guessing another fixed attempt count that might again be too short
// for some board.
float readSettledPosEstimate(Leg& leg, const std::string& joint,
                             std::chrono::milliseconds max_wait = std::chrono::milliseconds(15000)) {
    auto deadline = std::chrono::steady_clock::now() + max_wait;
    float prev = leg.getParamFloat(joint, EP_POS_ESTIMATE);
    int attempt = 1;
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        float cur = leg.getParamFloat(joint, EP_POS_ESTIMATE);
        ++attempt;
        bool agree = !std::isnan(cur) && !std::isnan(prev) && std::fabs(cur - prev) < 0.001f;
        if (agree && cur != 0.0f) {
            return cur;
        }
        if (agree) {
            std::cout << "  pos_estimate read exactly 0 twice in a row (attempt " << attempt
                      << ") — still could be a stuck timeout, not a real zero. Still waiting for the "
                         "board to finish recovering, keep holding..." << std::endl;
        } else {
            std::cout << "  pos_estimate read " << prev << " then " << cur << " turns (attempt " << attempt
                      << ") — didn't agree, still waiting, keep holding..." << std::endl;
        }
        prev = cur;
    }
    throw std::runtime_error(
        joint + ": pos_estimate never gave a trustworthy reading within " +
        std::to_string(max_wait.count()) + "ms (stuck at " + std::to_string(prev) + " turns) — this "
        "board is taking unusually long to recover from the reboot, or something else is wrong. You "
        "can let go of the joint now; check Serial output on its Teensy, then try --check on this "
        "joint once it responds normally before re-running the full calibration.");
}

void printReadback(Leg& leg, const std::string& joint) {
    std::cout << "Readback for " << joint << ":" << std::endl;
    std::cout << "  raw (absolute sensor)  = " << leg.getParamFloat(joint, EP_RAW_ABSOLUTE) << std::endl;
    std::cout << "  offset                 = " << leg.getParamFloat(joint, EP_OFFSET) << std::endl;
    std::cout << "  offset_valid           = " << leg.getParamBool(joint, EP_OFFSET_VALID) << std::endl;
    std::cout << "  approx_init_pos        = " << leg.getParamFloat(joint, EP_APPROX_INIT_POS) << std::endl;
    std::cout << "  approx_init_pos_valid  = " << leg.getParamBool(joint, EP_APPROX_INIT_POS_VALID) << std::endl;
    std::cout << "  absolute_setpoints     = " << leg.getParamBool(joint, EP_ABSOLUTE_SETPOINTS) << std::endl;
    std::cout << "  pos_rel                = " << leg.getParamFloat(joint, EP_POS_REL) << std::endl;
    std::cout << "  working_offset         = " << leg.getParamFloat(joint, EP_WORKING_OFFSET) << std::endl;
    std::cout << "  pos_estimate (CAN GET) = " << leg.getParamFloat(joint, EP_POS_ESTIMATE) << " turns" << std::endl;
    // Live position, via the *existing* feedback path (Get_Encoder_Estimates
    // over CAN -> SystemData -> getJointState) — not a new endpoint, this is
    // the same pos_estimate the rest of this project already reads at 500Hz.
    std::cout << "  live position_rad      = " << leg.getJointState(joint).position_rad
              << "  (should already be ~0 immediately after a fresh power-on, no homing needed)" << std::endl;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <joint_name>            (run the one-time calibration + save)\n"
                  << "  " << argv[0] << " <joint_name> --check    (read-only: verify without writing/rebooting)\n";
        return 1;
    }
    std::string joint = argv[1];
    bool check_only = (argc == 3 && std::string(argv[2]) == "--check");

    HardwareBridge bridge(false);
    bridge.start();
    std::cout << "Sending heartbeats for 1s to let the Teensy learn our IP..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(1000));

    int result = 0;
    try {
        Leg& leg = selectLeg(bridge, joint);

        // This board may have rebooted on its own (a previous run of this
        // tool, or a real physical power-cycle) since the Teensy last booted
        // and applied its CAN broadcast-rate config.
        restoreCanBroadcastRates(leg, joint);

        if (check_only) {
            printReadback(leg, joint);
            bridge.stop();
            return 0;
        }

        std::cout << "Calibrating " << joint << " to zero at its current physical position." << std::endl;
        std::cout << ">>> HOLD " << joint << " FIRMLY AT THE ZERO POSITION NOW — do not let go until this "
                  << "tool says it's safe. The absolute encoder tracks live position continuously, so any "
                  << "movement from this point until then (across both reboots if a second pass runs) "
                  << "shows up as calibration error, not a separate, fixable thing afterward. <<<" << std::endl;
        float raw = leg.getParamFloat(joint, EP_RAW_ABSOLUTE);
        std::cout << "  current raw absolute reading = " << raw << std::endl;

        std::cout << "First pass:" << std::endl;
        writeAndVerifyFloat(leg, joint, EP_OFFSET, raw, "offset");
        writeAndVerifyBool(leg, joint, EP_OFFSET_VALID, true, "offset_valid");
        writeAndVerifyFloat(leg, joint, EP_APPROX_INIT_POS, 0.0f, "approx_init_pos");
        writeAndVerifyBool(leg, joint, EP_APPROX_INIT_POS_VALID, true, "approx_init_pos_valid");
        writeAndVerifyBool(leg, joint, EP_ABSOLUTE_SETPOINTS, true, "absolute_setpoints");
        saveAndReboot(leg, joint);

        float error = readSettledPosEstimate(leg, joint);
        std::cout << "  post-reboot pos_estimate (error) = " << error << " turns" << std::endl;

        if (std::isnan(error) || std::fabs(error) > kZeroToleranceTurns) {
            std::cout << "Second pass: correcting offset by the observed error "
                      << "(offset_2 = offset_1 - error)... KEEP HOLDING " << joint << " AT ZERO."
                      << std::endl;
            if (std::isnan(error)) {
                std::cout << "  error is NaN, not a finite offset to correct with — stopping. "
                          << "Check --check output above for what went wrong." << std::endl;
                throw std::runtime_error("pos_estimate was NaN after first pass");
            }
            float corrected_offset = raw - error;
            writeAndVerifyFloat(leg, joint, EP_OFFSET, corrected_offset, "offset");
            saveAndReboot(leg, joint);

            error = readSettledPosEstimate(leg, joint);
            std::cout << "  post-reboot pos_estimate (error) = " << error << " turns" << std::endl;
        }

        std::cout << ">>> You can let go of " << joint << " now. <<<" << std::endl;
        std::cout << (std::fabs(error) <= kZeroToleranceTurns ? "PASS" : "FAIL")
                  << ": final pos_estimate = " << error << " turns (tolerance "
                  << kZeroToleranceTurns << ")" << std::endl;

        std::cout << "Final ";
        printReadback(leg, joint);
        std::cout << "This proves the calibration survived save_configuration()'s reboot. "
                  << "For full confidence across a real power cycle, physically power-cycle "
                  << "this board and run\n  " << argv[0] << " " << joint << " --check\nagain afterward."
                  << std::endl;

        if (std::fabs(error) > kZeroToleranceTurns) result = 1;
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        result = 1;
    }

    bridge.stop();
    return result;
}
