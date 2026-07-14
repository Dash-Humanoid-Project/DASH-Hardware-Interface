#pragma once
#include <map>
#include <string>
#include "Leg.h"
#include "LeftLegKinematics.h"

// Unified joint-space + Cartesian-space PD+feedforward primitive, modeled
// directly on Cheetah-Software's LegController (common/src/Controllers/
// LegController.cpp). Both term sets are always live and combined every
// call to updateCommand() — this is not a mode switch between "joint
// impedance" and "Cartesian impedance"; either can be zeroed out to recover
// the other's behavior, matching how Cheetah's real hardware runs both
// simultaneously.
//
// Key architectural point (see Phase 2 plan / conversation): joint-space
// tracking (qDes/qdDes/kpJoint/kdJoint) is executed LOCALLY by the ODrive's
// own onboard position/velocity cascade — this class never sends a raw
// override torque for the joint term. Only the Cartesian term is computed
// here on the PC and folded in as an additive feedforward torque via
// Leg::setPositions()'s torque_ff_nm parameter, exactly mirroring how
// Cheetah's updateCommand() writes Cartesian-derived torque into tau_ff
// while leaving qDes/kpJoint/kdJoint for the board to track locally.
//
// Cartesian terms are scoped to the 4-joint chain LeftLegKinematics.h
// defines (l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee) — same scope as the
// prior --cartesian test mode. Joint-space terms cover every joint on the
// wrapped Leg (including l_ankle), read generically from Leg::motorConfigs().
//
// Deliberately NOT included (out of scope for this phase): an edamp-style
// damping-only fallback. Trivial to add later (kpJoint=0, tauFeedForward=0,
// kdJoint=gain is already expressible) but not built now.
class LegController {
public:
    explicit LegController(Leg& leg);

    // ---------- Command setters ----------
    // Joint-space: must cover every joint on the leg (see Leg::setGains).
    void setJointGains(const std::map<std::string, float>& kp_joint,
                       const std::map<std::string, float>& kd_joint);
    void setJointTargets(const std::map<std::string, float>& q_des,
                         const std::map<std::string, float>& qd_des = {});
    void setJointFeedforward(const std::map<std::string, float>& tau_ff_nm);

    // Cartesian-space: applies to the l_hip_yaw/l_hip_roll/l_hip_pitch/l_knee chain only.
    void setCartesianGains(LeftLeg::Vec3 kp_cartesian, LeftLeg::Vec3 kd_cartesian);
    void setCartesianTargets(LeftLeg::Vec3 p_des, LeftLeg::Vec3 v_des = {0, 0, 0});
    void setCartesianFeedforward(LeftLeg::Vec3 force_ff);

    // Zeros all command state (gains, targets, feedforward) — safe starting
    // point before the first real setpoint is issued. Mirrors Cheetah's
    // LegControllerCommand::zero().
    void zeroCommand();

    // ---------- Data ----------
    // Refreshes q/qd (all joints) and p/v/J (Cartesian chain) from the
    // wrapped Leg's current feedback. Call once per control cycle before
    // updateCommand().
    void updateData();

    const std::map<std::string, float>& q() const  { return q_; }
    const std::map<std::string, float>& qd() const { return qd_; }
    LeftLeg::Vec3 p() const { return p_; }
    LeftLeg::Vec3 v() const { return v_; }
    const std::map<std::string, float>& tauEstimate() const { return tau_estimate_; }

    // ---------- Dispatch ----------
    // Computes the combined command (Cartesian PD -> J^T -> additive torque
    // feedforward, joint targets/gains sent for local ODrive tracking) and
    // sends it. Sends SetGainsCommand only when kpJoint/kdJoint actually
    // changed since the last call.
    void updateCommand();

private:
    Leg& leg_;

    // Joint-space command state, keyed by joint name (every joint on leg_).
    std::map<std::string, float> kp_joint_, kd_joint_;
    std::map<std::string, float> q_des_, qd_des_;
    std::map<std::string, float> tau_ff_joint_;

    // Cartesian-space command state (l_hip_yaw/l_hip_roll/l_hip_pitch/l_knee chain).
    LeftLeg::Vec3 kp_cartesian_{0, 0, 0}, kd_cartesian_{0, 0, 0};
    LeftLeg::Vec3 p_des_{0, 0, 0}, v_des_{0, 0, 0};
    LeftLeg::Vec3 force_ff_{0, 0, 0};

    // Data, refreshed by updateData().
    std::map<std::string, float> q_, qd_;
    LeftLeg::Vec3 p_{0, 0, 0}, v_{0, 0, 0};
    double J_[3][4] = {};
    std::map<std::string, float> tau_estimate_;

    // Last gains actually sent, to avoid resending SetGainsCommand every cycle.
    std::map<std::string, float> last_sent_kp_, last_sent_kd_;
    bool gains_sent_once_ = false;
};
