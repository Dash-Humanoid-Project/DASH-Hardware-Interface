// Generic one-off ODrive CAN parameter ("endpoint") reader/writer — a thin
// CLI wrapper around Leg::getParam*/setParam* (see Leg.h), which already
// implement the GetSetParam CAN path (see include/Command.h's
// GetSetParamCommand/ParamResponse and teensy2.ino's GetSetParam handler
// for the wire format).
//
// Use this to inspect or poke at any ODrive endpoint by numeric ID without
// writing dedicated code for it first — e.g. to check a joint's
// motor.config.current_lim / controller.config.torque_lim once you know
// their endpoint_id (get it via the odrive Python package, live:
// type(obj).__dict__[name]._info.endpoint_id — see Leg.h's own comment on
// getParamFloat for this). Endpoint IDs are firmware-build-specific but
// were confirmed shared across every board checked in this project so far
// (fw 0.6.11 / hw 4.4.58) — see CalibrateAbsoluteEncoder.cpp's EP_* constants
// for examples of IDs already known for this hardware.
//
// Recreates a tool that existed once before (build artifacts survived from
// 2026-09-10 under the name "param_test" with the same get/set CLI shape)
// but was never committed to git and had no surviving source — this version
// is committed so that doesn't happen again.
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "HardwareBridge.h"
#include "TestUtils.h"

namespace {

void printUsage(const char* prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " get <joint> <endpoint_id> <float|bool|uint8|int32> [timeout_ms]\n"
              << "  " << prog << " set <joint> <endpoint_id> <float|bool|uint8|int32> <value>\n"
              << "\n"
              << "<joint> is any joint name (l_hip_yaw, r_knee, l_elbow, ...) — resolved to its\n"
              << "owning limb automatically. <endpoint_id> is the ODrive's own numeric endpoint\n"
              << "id (not a position in any list this project defines).\n";
}

} // namespace

int main(int argc, char** argv) {
    bool sim_mode = false;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--sim") sim_mode = true;
        else args.push_back(a);
    }

    if (args.size() < 4) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string& op = args[0];
    const std::string& joint = args[1];
    uint16_t endpoint_id;
    try {
        endpoint_id = static_cast<uint16_t>(std::stoul(args[2]));
    } catch (const std::exception&) {
        std::cerr << "endpoint_id must be an integer, got: " << args[2] << std::endl;
        return 1;
    }
    const std::string& type = args[3];

    if (op != "get" && op != "set") {
        printUsage(argv[0]);
        return 1;
    }
    if (type != "float" && type != "bool" && type != "uint8" && type != "int32") {
        std::cerr << "type must be one of float|bool|uint8|int32, got: " << type << std::endl;
        return 1;
    }
    if (op == "get" && args.size() > 5) { printUsage(argv[0]); return 1; }
    if (op == "set" && args.size() != 5) { printUsage(argv[0]); return 1; }

    HardwareBridge bridge(sim_mode);
    bridge.start();
    std::cout << "Sending heartbeats for 1s to let the Teensy learn our IP..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(1000));

    int result = 0;
    try {
        Leg& leg = selectLeg(bridge, joint);

        if (op == "get") {
            int timeout_ms = args.size() == 5 ? std::stoi(args[4]) : 200;
            if (type == "float") {
                std::cout << joint << " endpoint " << endpoint_id << " (float) = "
                          << leg.getParamFloat(joint, endpoint_id, timeout_ms) << std::endl;
            } else if (type == "bool") {
                std::cout << joint << " endpoint " << endpoint_id << " (bool) = "
                          << (leg.getParamBool(joint, endpoint_id, timeout_ms) ? "true" : "false") << std::endl;
            } else if (type == "uint8") {
                std::cout << joint << " endpoint " << endpoint_id << " (uint8) = "
                          << static_cast<int>(leg.getParamUint8(joint, endpoint_id, timeout_ms)) << std::endl;
            } else {
                std::cout << joint << " endpoint " << endpoint_id << " (int32) = "
                          << leg.getParamInt32(joint, endpoint_id, timeout_ms) << std::endl;
            }
        } else {
            const std::string& value_str = args[4];
            if (type == "float") {
                float v = std::stof(value_str);
                leg.setParamFloat(joint, endpoint_id, v);
                std::cout << "Set " << joint << " endpoint " << endpoint_id << " (float) = " << v << std::endl;
            } else if (type == "bool") {
                bool v = (value_str == "1" || value_str == "true");
                leg.setParamBool(joint, endpoint_id, v);
                std::cout << "Set " << joint << " endpoint " << endpoint_id << " (bool) = "
                          << (v ? "true" : "false") << std::endl;
            } else if (type == "uint8") {
                uint8_t v = static_cast<uint8_t>(std::stoul(value_str));
                leg.setParamUint8(joint, endpoint_id, v);
                std::cout << "Set " << joint << " endpoint " << endpoint_id << " (uint8) = "
                          << static_cast<int>(v) << std::endl;
            } else {
                int32_t v = std::stol(value_str);
                leg.setParamInt32(joint, endpoint_id, v);
                std::cout << "Set " << joint << " endpoint " << endpoint_id << " (int32) = " << v << std::endl;
            }
            std::cout << "(fire-and-forget — follow up with a 'get' of the same endpoint to verify)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        result = 1;
    }

    bridge.stop();
    return result;
}
