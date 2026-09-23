#include "TrajectoryPlayback.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include "KeyboardInput.h"
#include "PeriodicTimer.h"
#include "TestUtils.h"

namespace {
const char* kJointSuffixes[4] = {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"};

std::array<std::string, 4> armJointNames(const std::string& prefix) {
    return {prefix + kJointSuffixes[0], prefix + kJointSuffixes[1],
            prefix + kJointSuffixes[2], prefix + kJointSuffixes[3]};
}

constexpr double RECORD_PERIOD_S = 0.02;  // 50 Hz — plenty for hand-guided motion, keeps files small
constexpr double PLAY_TICK_S     = 0.01;  // 100 Hz PC-side playback tick

// Ramp-in duration scales with how far the arm actually has to travel to
// reach the trajectory's start, rather than always taking the same fixed
// time — running --playback right after --record (arm hasn't moved) would
// otherwise waste a full fixed duration approaching a target it's already
// sitting at. RAMP_SPEED_RAD_S is a conservative nominal approach speed;
// MIN/MAX bound it so a tiny gap doesn't snap and a huge one doesn't ramp
// forever.
constexpr double RAMP_SPEED_RAD_S = 1.0;
constexpr double RAMP_MIN_S       = 0.3;
constexpr double RAMP_MAX_S       = 2.0;

double rampDurationFor(const float start_q[4], const float target_q[4]) {
    float max_delta = 0.0f;
    for (int i = 0; i < 4; ++i)
        max_delta = std::max(max_delta, std::fabs(target_q[i] - start_q[i]));
    double dur = static_cast<double>(max_delta) / RAMP_SPEED_RAD_S;
    return std::clamp(dur, RAMP_MIN_S, RAMP_MAX_S);
}

// How far (rad, any single joint) a sample must differ from the recording's
// first sample to count as "moving" rather than encoder noise / hand tremor
// while holding still. Not derived from any swept-motion data — arm joints
// have none — picked as a small fraction of a degree above typical encoder
// noise.
constexpr float MOVEMENT_THRESHOLD_RAD = 0.02f;

// Local ODrive position/velocity gains explicitly set before playback,
// rather than trusting whatever's already configured (unlike --position
// mode's gentle slow sweep, which apparently tracks fine on ambient gains —
// this trajectory is faster and larger, e.g. shoulder_pitch fighting
// gravity through ~2 rad). NOT independently tuned/verified on hardware.
//
// History: started at 10.0/0.15, then 20.0/0.25, both reported as
// undershooting shoulder_pitch without reported oscillation/instability.
// Then found teensy3.ino's SetGains handler had the same TX-drop bug as
// idleAllODrives() (now fixed) — since gain delivery wasn't reliable before
// that fix, neither prior value was ever confirmed to have actually landed.
// Backed all the way down to 2.0/0.1 (ImpedanceMode's own already-run
// placeholder) to re-test with reliable delivery — confirmed noticeably
// better tracking than before, validating that the delivery bug was real,
// but still not stiff enough. Stepping up to the next deliberate increment,
// 10.0/0.15 — the first value tried before the delivery bug was found, now
// actually confirmed to reach the ODrive this time. If still undershooting
// with no oscillation, try 20.0/0.25 next. If undershoot persists even once
// a meaningfully higher value is confirmed both delivered AND
// non-oscillating, the next suspect is the ODrive's own current_lim/
// torque_lim — a separate hardware config this codebase has no path to
// touch at all.
constexpr float PLAY_POS_GAIN = 10.0f;
constexpr float PLAY_VEL_GAIN = 0.15f;

bool differsRaw(const float a[4], const float b[4]) {
    for (int i = 0; i < 4; ++i)
        if (std::fabs(a[i] - b[i]) > MOVEMENT_THRESHOLD_RAD) return true;
    return false;
}

bool differs(const TrajectorySample& a, const TrajectorySample& b) {
    return differsRaw(a.q, b.q);
}

bool differsDual(const DualArmSample& a, const DualArmSample& b) {
    return differsRaw(a.q_left, b.q_left) || differsRaw(a.q_right, b.q_right);
}

// Trims leading/trailing frames that haven't moved from the very first
// sample — relies on the assumed record workflow (hold still, move, return
// near the start) rather than a generic "stopped moving" velocity check,
// since a slow hand-guided return could otherwise look like "still moving"
// right up until the end.
std::vector<TrajectorySample> trimStaticEnds(const std::vector<TrajectorySample>& raw) {
    if (raw.size() < 2) return {};

    size_t first = 0;
    while (first < raw.size() && !differs(raw[first], raw[0])) ++first;
    if (first == raw.size()) return {};  // never moved
    if (first > 0) --first;              // keep one static frame before motion starts

    size_t last = raw.size() - 1;
    while (last > first && !differs(raw[last], raw[0])) --last;
    if (last + 1 < raw.size()) ++last;   // keep one static frame after motion ends

    std::vector<TrajectorySample> out(raw.begin() + first, raw.begin() + last + 1);
    double t0 = out.front().t;
    for (auto& s : out) s.t -= t0;
    return out;
}

// Same trim, but triggers on either arm moving — a single-arm gesture
// during an otherwise-bimanual recording still counts as the start/end of
// motion.
std::vector<DualArmSample> trimStaticEndsDual(const std::vector<DualArmSample>& raw) {
    if (raw.size() < 2) return {};

    size_t first = 0;
    while (first < raw.size() && !differsDual(raw[first], raw[0])) ++first;
    if (first == raw.size()) return {};
    if (first > 0) --first;

    size_t last = raw.size() - 1;
    while (last > first && !differsDual(raw[last], raw[0])) --last;
    if (last + 1 < raw.size()) ++last;

    std::vector<DualArmSample> out(raw.begin() + first, raw.begin() + last + 1);
    double t0 = out.front().t;
    for (auto& s : out) s.t -= t0;
    return out;
}

float median3(float a, float b, float c) {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

// Rejects single-sample glitches (a momentary bad CAN/encoder read that
// snaps back on the very next sample) without touching genuine multi-sample
// motion — deliberately NOT the moving-average tried and reverted earlier
// (see git history / project memory): an average blurs real sustained
// peaks/holds, a median of 3 doesn't (median of [x,x,x] is x, unaffected;
// only an isolated outlier in the middle gets replaced). This matters
// specifically for the velocity feedforward playArmTrajectory() computes
// via finite difference — a single glitched sample doesn't just look like
// one bad position, it also creates two huge spurious velocity spikes
// (into the glitch and back out of it) on either side of it.
std::vector<TrajectorySample> medianFilter3(const std::vector<TrajectorySample>& in) {
    if (in.size() < 3) return in;
    std::vector<TrajectorySample> out = in;
    for (size_t i = 1; i + 1 < in.size(); ++i)
        for (int j = 0; j < 4; ++j)
            out[i].q[j] = median3(in[i - 1].q[j], in[i].q[j], in[i + 1].q[j]);
    return out;
}

std::vector<DualArmSample> medianFilter3Dual(const std::vector<DualArmSample>& in) {
    if (in.size() < 3) return in;
    std::vector<DualArmSample> out = in;
    for (size_t i = 1; i + 1 < in.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i].q_left[j]  = median3(in[i - 1].q_left[j],  in[i].q_left[j],  in[i + 1].q_left[j]);
            out[i].q_right[j] = median3(in[i - 1].q_right[j], in[i].q_right[j], in[i + 1].q_right[j]);
        }
    }
    return out;
}

std::map<std::string, float> jointMap(const std::array<std::string, 4>& names, const float q[4]) {
    return {
        {names[0], q[0]}, {names[1], q[1]},
        {names[2], q[2]}, {names[3], q[3]},
    };
}
} // namespace

std::vector<TrajectorySample> recordArmTrajectory(HardwareBridge& bridge, Leg& arm, const std::string& prefix)
{
    auto joint_names = armJointNames(prefix);

    std::cout << "Idling arm for hand-guided recording..." << std::endl;
    arm.idle();

    std::cout << "Arm is backdrivable. Guide it through the desired motion and\n"
                 "return it close to its starting position. Press any key when done."
              << std::endl;

    KeyboardInput keyboard;
    PeriodicTimer record_timer(RECORD_PERIOD_S);
    std::vector<TrajectorySample> raw;
    auto t0 = std::chrono::steady_clock::now();

    while (!shutdown_requested) {
        if (keyboard.pollKey() >= 0) break;

        auto js = arm.getJointStates();
        TrajectorySample s;
        s.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        for (int i = 0; i < 4; ++i) s.q[i] = js[joint_names[i]].position_rad;
        raw.push_back(s);

        bridge.sendHeartbeat();
        record_timer.wait();
    }

    std::cout << "Recorded " << raw.size() << " raw samples." << std::endl;
    auto trimmed = trimStaticEnds(raw);
    std::cout << "Trimmed to " << trimmed.size() << " samples ("
              << (trimmed.empty() ? 0.0 : trimmed.back().t) << "s of motion)." << std::endl;
    return medianFilter3(trimmed);
}

void saveTrajectoryCSV(const std::vector<TrajectorySample>& traj, const std::string& path, const std::string& prefix)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cout << "Unable to open " << path << " for writing." << std::endl;
        return;
    }
    auto joint_names = armJointNames(prefix);
    f << "t_s," << joint_names[0] << "_rad," << joint_names[1] << "_rad,"
      << joint_names[2] << "_rad," << joint_names[3] << "_rad\n";
    for (const auto& s : traj)
        f << s.t << "," << s.q[0] << "," << s.q[1] << "," << s.q[2] << "," << s.q[3] << "\n";
    std::cout << "Saved " << traj.size() << " samples to " << path << std::endl;
}

std::vector<TrajectorySample> loadTrajectoryCSV(const std::string& path)
{
    std::vector<TrajectorySample> out;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cout << "Unable to open " << path << " for reading." << std::endl;
        return out;
    }
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string field;
        TrajectorySample s{};
        std::getline(ss, field, ','); s.t = std::stod(field);
        for (int i = 0; i < 4; ++i) { std::getline(ss, field, ','); s.q[i] = std::stof(field); }
        out.push_back(s);
    }
    std::cout << "Loaded " << out.size() << " samples from " << path << std::endl;
    return out;
}

void playArmTrajectory(HardwareBridge& bridge, Leg& arm, const std::vector<TrajectorySample>& traj,
                       const std::string& prefix)
{
    if (traj.empty()) {
        std::cout << "Empty trajectory, nothing to play." << std::endl;
        return;
    }

    auto joint_names = armJointNames(prefix);

    std::cout << "Arming arm for playback..." << std::endl;
    arm.startClosedLoop();
    std::cout << "Waiting for encoder feedback..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(500));

    std::map<std::string, float> pos_gains, vel_gains;
    for (int i = 0; i < 4; ++i) {
        pos_gains[joint_names[i]] = PLAY_POS_GAIN;
        vel_gains[joint_names[i]] = PLAY_VEL_GAIN;
    }
    arm.setGains(pos_gains, vel_gains);
    std::cout << "Set tracking gains: pos_gain=" << PLAY_POS_GAIN
              << " vel_gain=" << PLAY_VEL_GAIN << std::endl;

    // Ramp smoothly from wherever the arm currently is to the trajectory's
    // first sample, rather than snapping straight to it.
    auto js = arm.getJointStates();
    float start_q[4];
    for (int i = 0; i < 4; ++i) start_q[i] = js[joint_names[i]].position_rad;

    double ramp_dur = rampDurationFor(start_q, traj[0].q);
    std::cout << "Ramping to trajectory start over " << ramp_dur << "s..." << std::endl;
    PeriodicTimer ramp_timer(PLAY_TICK_S);
    auto ramp_t0 = std::chrono::steady_clock::now();
    while (!shutdown_requested) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_t0).count();
        if (elapsed >= ramp_dur) break;
        float p = static_cast<float>(elapsed / ramp_dur);
        float q[4];
        for (int i = 0; i < 4; ++i) q[i] = start_q[i] + (traj[0].q[i] - start_q[i]) * p;
        arm.setPositions(jointMap(joint_names, q));
        ramp_timer.wait();
    }
    if (shutdown_requested) return;

    std::cout << "Playing back " << traj.size() << " samples over "
              << traj.back().t << "s..." << std::endl;

    PeriodicTimer play_timer(PLAY_TICK_S);
    auto play_t0 = std::chrono::steady_clock::now();
    size_t idx = 0;
    while (!shutdown_requested) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - play_t0).count();
        while (idx + 1 < traj.size() && traj[idx + 1].t <= elapsed) ++idx;

        // Linearly interpolate the position target between traj[idx] and
        // traj[idx+1] rather than holding traj[idx].q constant until idx
        // jumps — a static target fighting a nonzero velocity feedforward
        // (below) made the ODrive's local PD alternately snap back to a
        // stale hold-point and get shoved forward by the FF term, instead
        // of smoothly tracking the recorded motion.
        float q_cmd[4];
        float vel_ff[4] = {0, 0, 0, 0};
        if (idx + 1 < traj.size()) {
            double dt = traj[idx + 1].t - traj[idx].t;
            float frac = dt > 1e-4 ? static_cast<float>((elapsed - traj[idx].t) / dt) : 0.0f;
            frac = std::clamp(frac, 0.0f, 1.0f);
            for (int i = 0; i < 4; ++i) {
                q_cmd[i] = traj[idx].q[i] + (traj[idx + 1].q[i] - traj[idx].q[i]) * frac;
                if (dt > 1e-4) vel_ff[i] = static_cast<float>((traj[idx + 1].q[i] - traj[idx].q[i]) / dt);
            }
        } else {
            for (int i = 0; i < 4; ++i) q_cmd[i] = traj[idx].q[i];
        }
        arm.setPositions(jointMap(joint_names, q_cmd), jointMap(joint_names, vel_ff));

        if (idx == traj.size() - 1 && elapsed >= traj.back().t) break;
        play_timer.wait();
    }

    // Hold at the final recorded position rather than immediately releasing.
    arm.setPositions(jointMap(joint_names, traj.back().q));
    std::cout << "Playback complete." << std::endl;
}

std::vector<DualArmSample> recordBothArmsTrajectory(HardwareBridge& bridge, Leg& left_arm, Leg& right_arm)
{
    auto left_names = armJointNames("l_");
    auto right_names = armJointNames("r_");

    std::cout << "Idling both arms for hand-guided recording..." << std::endl;
    left_arm.idle();
    right_arm.idle();

    std::cout << "Both arms are backdrivable. Guide them through the desired motion and\n"
                 "return them close to their starting positions. Press any key when done."
              << std::endl;

    KeyboardInput keyboard;
    PeriodicTimer record_timer(RECORD_PERIOD_S);
    std::vector<DualArmSample> raw;
    auto t0 = std::chrono::steady_clock::now();

    while (!shutdown_requested) {
        if (keyboard.pollKey() >= 0) break;

        auto jl = left_arm.getJointStates();
        auto jr = right_arm.getJointStates();
        DualArmSample s;
        s.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        for (int i = 0; i < 4; ++i) {
            s.q_left[i]  = jl[left_names[i]].position_rad;
            s.q_right[i] = jr[right_names[i]].position_rad;
        }
        raw.push_back(s);

        bridge.sendHeartbeat();
        record_timer.wait();
    }

    std::cout << "Recorded " << raw.size() << " raw samples." << std::endl;
    auto trimmed = trimStaticEndsDual(raw);
    std::cout << "Trimmed to " << trimmed.size() << " samples ("
              << (trimmed.empty() ? 0.0 : trimmed.back().t) << "s of motion)." << std::endl;
    return medianFilter3Dual(trimmed);
}

void saveDualArmTrajectoryCSV(const std::vector<DualArmSample>& traj, const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cout << "Unable to open " << path << " for writing." << std::endl;
        return;
    }
    auto left_names = armJointNames("l_");
    auto right_names = armJointNames("r_");
    f << "t_s";
    for (const auto& n : left_names)  f << "," << n << "_rad";
    for (const auto& n : right_names) f << "," << n << "_rad";
    f << "\n";
    for (const auto& s : traj) {
        f << s.t;
        for (int i = 0; i < 4; ++i) f << "," << s.q_left[i];
        for (int i = 0; i < 4; ++i) f << "," << s.q_right[i];
        f << "\n";
    }
    std::cout << "Saved " << traj.size() << " samples to " << path << std::endl;
}

std::vector<DualArmSample> loadDualArmTrajectoryCSV(const std::string& path)
{
    std::vector<DualArmSample> out;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cout << "Unable to open " << path << " for reading." << std::endl;
        return out;
    }
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string field;
        DualArmSample s{};
        std::getline(ss, field, ','); s.t = std::stod(field);
        for (int i = 0; i < 4; ++i) { std::getline(ss, field, ','); s.q_left[i]  = std::stof(field); }
        for (int i = 0; i < 4; ++i) { std::getline(ss, field, ','); s.q_right[i] = std::stof(field); }
        out.push_back(s);
    }
    std::cout << "Loaded " << out.size() << " samples from " << path << std::endl;
    return out;
}

void playBothArmsTrajectory(HardwareBridge& bridge, Leg& left_arm, Leg& right_arm,
                            const std::vector<DualArmSample>& traj)
{
    if (traj.empty()) {
        std::cout << "Empty trajectory, nothing to play." << std::endl;
        return;
    }

    auto left_names = armJointNames("l_");
    auto right_names = armJointNames("r_");

    std::cout << "Arming both arms for playback..." << std::endl;
    left_arm.startClosedLoop();
    right_arm.startClosedLoop();
    std::cout << "Waiting for encoder feedback..." << std::endl;
    sendKeepAliveFor(bridge, std::chrono::milliseconds(500));

    std::map<std::string, float> pos_gains_left, vel_gains_left, pos_gains_right, vel_gains_right;
    for (int i = 0; i < 4; ++i) {
        pos_gains_left[left_names[i]]   = PLAY_POS_GAIN;
        vel_gains_left[left_names[i]]   = PLAY_VEL_GAIN;
        pos_gains_right[right_names[i]] = PLAY_POS_GAIN;
        vel_gains_right[right_names[i]] = PLAY_VEL_GAIN;
    }
    left_arm.setGains(pos_gains_left, vel_gains_left);
    right_arm.setGains(pos_gains_right, vel_gains_right);
    std::cout << "Set tracking gains: pos_gain=" << PLAY_POS_GAIN
              << " vel_gain=" << PLAY_VEL_GAIN << " (both arms)" << std::endl;

    // Ramp both arms smoothly from wherever they currently are to the
    // trajectory's first sample, together — the longer of the two arms'
    // individually-computed durations governs, so neither arm gets rushed
    // to keep pace with the other.
    auto jl = left_arm.getJointStates();
    auto jr = right_arm.getJointStates();
    float start_left[4], start_right[4];
    for (int i = 0; i < 4; ++i) {
        start_left[i]  = jl[left_names[i]].position_rad;
        start_right[i] = jr[right_names[i]].position_rad;
    }

    double ramp_dur = std::max(rampDurationFor(start_left, traj[0].q_left),
                                rampDurationFor(start_right, traj[0].q_right));
    std::cout << "Ramping to trajectory start over " << ramp_dur << "s..." << std::endl;
    PeriodicTimer ramp_timer(PLAY_TICK_S);
    auto ramp_t0 = std::chrono::steady_clock::now();
    while (!shutdown_requested) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_t0).count();
        if (elapsed >= ramp_dur) break;
        float p = static_cast<float>(elapsed / ramp_dur);
        float ql[4], qr[4];
        for (int i = 0; i < 4; ++i) {
            ql[i] = start_left[i]  + (traj[0].q_left[i]  - start_left[i])  * p;
            qr[i] = start_right[i] + (traj[0].q_right[i] - start_right[i]) * p;
        }
        left_arm.setPositions(jointMap(left_names, ql));
        right_arm.setPositions(jointMap(right_names, qr));
        ramp_timer.wait();
    }
    if (shutdown_requested) return;

    std::cout << "Playing back " << traj.size() << " samples over "
              << traj.back().t << "s (both arms)..." << std::endl;

    PeriodicTimer play_timer(PLAY_TICK_S);
    auto play_t0 = std::chrono::steady_clock::now();
    size_t idx = 0;
    while (!shutdown_requested) {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - play_t0).count();
        while (idx + 1 < traj.size() && traj[idx + 1].t <= elapsed) ++idx;

        // Same static-target-vs-velocity-feedforward interpolation fix as
        // playArmTrajectory, applied to both arms on the shared idx/elapsed.
        float ql_cmd[4], qr_cmd[4];
        float vl_ff[4] = {0, 0, 0, 0};
        float vr_ff[4] = {0, 0, 0, 0};
        if (idx + 1 < traj.size()) {
            double dt = traj[idx + 1].t - traj[idx].t;
            float frac = dt > 1e-4 ? static_cast<float>((elapsed - traj[idx].t) / dt) : 0.0f;
            frac = std::clamp(frac, 0.0f, 1.0f);
            for (int i = 0; i < 4; ++i) {
                ql_cmd[i] = traj[idx].q_left[i]  + (traj[idx + 1].q_left[i]  - traj[idx].q_left[i])  * frac;
                qr_cmd[i] = traj[idx].q_right[i] + (traj[idx + 1].q_right[i] - traj[idx].q_right[i]) * frac;
                if (dt > 1e-4) {
                    vl_ff[i] = static_cast<float>((traj[idx + 1].q_left[i]  - traj[idx].q_left[i])  / dt);
                    vr_ff[i] = static_cast<float>((traj[idx + 1].q_right[i] - traj[idx].q_right[i]) / dt);
                }
            }
        } else {
            for (int i = 0; i < 4; ++i) {
                ql_cmd[i] = traj[idx].q_left[i];
                qr_cmd[i] = traj[idx].q_right[i];
            }
        }
        left_arm.setPositions(jointMap(left_names, ql_cmd), jointMap(left_names, vl_ff));
        right_arm.setPositions(jointMap(right_names, qr_cmd), jointMap(right_names, vr_ff));

        if (idx == traj.size() - 1 && elapsed >= traj.back().t) break;
        play_timer.wait();
    }

    // Hold at the final recorded position rather than immediately releasing.
    left_arm.setPositions(jointMap(left_names, traj.back().q_left));
    right_arm.setPositions(jointMap(right_names, traj.back().q_right));
    std::cout << "Playback complete." << std::endl;
}
