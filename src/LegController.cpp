#include "LegController.h"

namespace {
// Joint order matches LeftLegKinematics.h's q[0..3] convention exactly
// (l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee) — the Cartesian chain this
// class supports. l_ankle is a valid joint-space-only target but plays no
// part in the Cartesian terms, same scope as the prior --cartesian mode.
constexpr const char* kCartesianJoints[4] = {"l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee"};

float mapGet(const std::map<std::string, float>& m, const std::string& key, float def = 0.0f) {
    auto it = m.find(key);
    return it != m.end() ? it->second : def;
}
}

LegController::LegController(Leg& leg) : leg_(leg) {}

void LegController::setJointGains(const std::map<std::string, float>& kp_joint,
                                  const std::map<std::string, float>& kd_joint) {
    kp_joint_ = kp_joint;
    kd_joint_ = kd_joint;
}

void LegController::setJointTargets(const std::map<std::string, float>& q_des,
                                    const std::map<std::string, float>& qd_des) {
    q_des_ = q_des;
    qd_des_ = qd_des;
}

void LegController::setJointFeedforward(const std::map<std::string, float>& tau_ff_nm) {
    tau_ff_joint_ = tau_ff_nm;
}

void LegController::setCartesianGains(LeftLeg::Vec3 kp_cartesian, LeftLeg::Vec3 kd_cartesian) {
    kp_cartesian_ = kp_cartesian;
    kd_cartesian_ = kd_cartesian;
}

void LegController::setCartesianTargets(LeftLeg::Vec3 p_des, LeftLeg::Vec3 v_des) {
    p_des_ = p_des;
    v_des_ = v_des;
}

void LegController::setCartesianFeedforward(LeftLeg::Vec3 force_ff) {
    force_ff_ = force_ff;
}

void LegController::zeroCommand() {
    kp_joint_.clear();
    kd_joint_.clear();
    q_des_.clear();
    qd_des_.clear();
    tau_ff_joint_.clear();
    kp_cartesian_ = {0, 0, 0};
    kd_cartesian_ = {0, 0, 0};
    p_des_ = {0, 0, 0};
    v_des_ = {0, 0, 0};
    force_ff_ = {0, 0, 0};
    // Matches Cheetah's LegControllerCommand::zero() semantics: this only
    // resets internal state. Call updateCommand() afterward for it to
    // actually reach the leg.
}

void LegController::updateData() {
    auto states = leg_.getJointStates();
    q_.clear();
    qd_.clear();
    for (const auto& kv : states) {
        q_[kv.first] = kv.second.position_rad;
        qd_[kv.first] = kv.second.velocity_rad_s;
    }

    double q4[4] = {
        q_.at(kCartesianJoints[0]), q_.at(kCartesianJoints[1]),
        q_.at(kCartesianJoints[2]), q_.at(kCartesianJoints[3]),
    };
    double qd4[4] = {
        qd_.at(kCartesianJoints[0]), qd_.at(kCartesianJoints[1]),
        qd_.at(kCartesianJoints[2]), qd_.at(kCartesianJoints[3]),
    };

    p_ = LeftLeg::forwardKinematics(q4);
    LeftLeg::computeJacobian(q4, J_);

    v_.x = J_[0][0]*qd4[0] + J_[0][1]*qd4[1] + J_[0][2]*qd4[2] + J_[0][3]*qd4[3];
    v_.y = J_[1][0]*qd4[0] + J_[1][1]*qd4[1] + J_[1][2]*qd4[2] + J_[1][3]*qd4[3];
    v_.z = J_[2][0]*qd4[0] + J_[2][1]*qd4[1] + J_[2][2]*qd4[2] + J_[2][3]*qd4[3];
}

void LegController::updateCommand() {
    // Cartesian PD + feedforward -> foot force -> joint torque via J^T.
    // This is the only term computed on the PC; joint-space tracking
    // (qDes/kpJoint/kdJoint below) stays local to the ODrive.
    LeftLeg::Vec3 pos_err = p_des_ - p_;
    LeftLeg::Vec3 vel_err = v_des_ - v_;
    LeftLeg::Vec3 foot_force = force_ff_
        + LeftLeg::Vec3{kp_cartesian_.x * pos_err.x, kp_cartesian_.y * pos_err.y, kp_cartesian_.z * pos_err.z}
        + LeftLeg::Vec3{kd_cartesian_.x * vel_err.x, kd_cartesian_.y * vel_err.y, kd_cartesian_.z * vel_err.z};

    double F[3] = {foot_force.x, foot_force.y, foot_force.z};
    double tau_cartesian[4];
    LeftLeg::jacobianTransposeMultiply(J_, F, tau_cartesian);

    std::map<std::string, float> tau_ff_total = tau_ff_joint_;
    for (int i = 0; i < 4; ++i) {
        float existing = mapGet(tau_ff_total, kCartesianJoints[i]);
        tau_ff_total[kCartesianJoints[i]] = existing + static_cast<float>(tau_cartesian[i]);
    }

    // Diagnostic-only estimate of the resulting torque — not measured,
    // computed from the command + last-known q/qd, matching Cheetah's exact
    // LegController.cpp formula.
    tau_estimate_.clear();
    for (const auto& mc : leg_.motorConfigs()) {
        const std::string& name = mc.joint_name;
        float ff = mapGet(tau_ff_total, name);
        float kp = mapGet(kp_joint_, name);
        float kd = mapGet(kd_joint_, name);
        float qdes = mapGet(q_des_, name);
        float qddes = mapGet(qd_des_, name);
        float qcur = mapGet(q_, name);
        float qdcur = mapGet(qd_, name);
        tau_estimate_[name] = ff + kp * (qdes - qcur) + kd * (qddes - qdcur);
    }

    // Only resend gains when they've actually changed — SetGainsCommand
    // isn't meant to go out every 2ms cycle the way position commands do.
    if (!gains_sent_once_ || kp_joint_ != last_sent_kp_ || kd_joint_ != last_sent_kd_) {
        leg_.setGains(kp_joint_, kd_joint_);
        last_sent_kp_ = kp_joint_;
        last_sent_kd_ = kd_joint_;
        gains_sent_once_ = true;
    }

    leg_.setPositions(q_des_, qd_des_, tau_ff_total);
}
