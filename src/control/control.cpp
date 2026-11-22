#include "control/control.hpp"
#include <cmath>
#include <cstdio>
#include <mujoco/mjtype.h>
#include <mujoco/mujoco.h>
#include <numbers>
#include <stdexcept>

namespace {
struct JointTarget {
    const char* joint;
    double target;
};

// The neutral stand pose, per joint, in radians. Positive pitch rotates a leg
// segment downward, negative upward: the femur rears up 75 deg, the tibia
// folds down 160 deg to plant the claw — knees high, trunk lifted.
constexpr double kStandCoxa = 0.0;
constexpr double kStandFemur = -1.31;
constexpr double kStandTibia = 2.79;
constexpr JointTarget kStandPose[]{{"coxa", kStandCoxa}, {"femur", kStandFemur}, {"tibia", kStandTibia}};

constexpr double kSwingLift = 0.6;           // lower-lweg fold at mid-swing (radians)
constexpr double kNominalStandHeight = 0.16; // trunk height of the sxtand pose
constexpr double kStanceFraction = 0.5;      // tripod duty factor: half the cycle stance, half swing
} // namespace

Control::Control(World& world) : world_(world) {
    for (int legId = 0; legId < kLegCount; ++legId) {
        char legName[16];
        std::snprintf(legName, sizeof(legName), "leg%d", legId);
        const int legBody = mj_name2id(&world_.model(), mjOBJ_BODY, legName);

        char femurName[16];
        std::snprintf(femurName, sizeof(femurName), "leg%d_femur", legId);
        const int femurBody = mj_name2id(&world_.model(), mjOBJ_BODY, femurName);

        char tibiaName[16];
        std::snprintf(tibiaName, sizeof(tibiaName), "leg%d_tibia", legId);
        const int tibiaBody = mj_name2id(&world_.model(), mjOBJ_BODY, tibiaName);

        char footName[16];
    std:
        snprintf(footName, sizeof(footName), "leg%d_foot", legId);
        const int footGeometry = mj_name2id(&world_.model(), mjOBJ_GEOM, footName);

        bool is_missing_body_component = legBody < 0 || femurBody < 0 || tibiaBody < 0 || footGeometry < 0;
        if (is_missing_body_component)
            throw std::runtime_error("model is missing some leg bodies or feet component");

        // mount direction at spawn;
        const mjtNum mountForward[3] = {1, 0, 0};
        mjtNum mountForwardWorld[3] = {0, 0, 0};
        mju_rotVecQuat(mountForwardWorld, mountForward, &world_.data().xquat[4 * legBody]);
        sweepSign_[legId] = mountForwardWorld[1] > 0.0 ? -1.0 : 1.0;

        // stance reach: horizonal coxa-axis-to-foot distance in the stand pose.
        const double coxaLength = world_.model().body_pos[3 * femurBody];
        const double femurLength = world_.model().body_pos[3 * tibiaBody];
        const double tibiaLength = world_.model().geom_pos[3 * footGeometry];

        footReach_[legId] = coxaLength + femurLength * std::cos(kStandFemur) + tibiaLength * std::cos(kStandFemur + kStandTibia);
    }
}

void Control::holdStandPose() {
    is_walking_ = false;
    for (int legId = 0; legId < kLegCount; ++legId) {
        for (const JointTarget& joint : kStandPose) {
            char actuatorName[32];
            std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_%s", legId, joint.joint);
            world_.setActuatorTarget(actuatorName, joint.target);
        }
    }
}

void Control::startWalking() {
    is_walking_ = true;
    gaitPhase_ = 0.0;
}

void Control::update() {
    if (!is_walking_)
        return;

    // Gait phase is Control's own clock accumulated one step at time
    // thus live frequency change bends the trajectory from where it is instead of teleporting it by (new frequency x elapsed time)
    const double cycles = gaitPhase_;
    const double heightScale = parameters_.stanceHeight / kNominalStandHeight;

    for (int legId = 0; legId < kLegCount; ++legId) {
        // alternating legs for tripod gait where front-left, mid-right, and rear-left go forward,
        // then front-right, mid-left, and rear-right.
        const double legPhase = std::fmod(cycles + (legId % 2) * 0.5, 1.0);

        // yaw sweep angle where stride = 2 * reach * sin(A) -> A = arcsin(stride/(2*reach))
        const double amplitude = std::asin(std::min(parameters_.stride / (2 * footReach_[legId]), 1.0));

        const double coxa = sweepSign_[legId] * amplitude * std::cos(2.0 * std::numbers::pi * legPhase);
        const double femur = kStandFemur * heightScale;
        double tibia = kStandTibia * heightScale;

        if (legPhase >= kStanceFraction) {
            const double swingProgress = (legPhase - kStanceFraction) / (1.0 - kStanceFraction);
            tibia -= kSwingLift * std::sin(std::numbers::pi * swingProgress);
        }

        writeLegTargets(legId, coxa, femur, tibia);
    }
    gaitPhase_ += world_.timestep() * parameters_.frequency * parameters_.speed;
}

void Control::writeLegTargets(int legId, double coxa, double femur, double tibia) {
    char actuatorName[32];
    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_coxa", legId);
    world_.setActuatorTarget(actuatorName, coxa);

    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_femur", legId);
    world_.setActuatorTarget(actuatorName, femur);

    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_tibia", legId);
    world_.setActuatorTarget(actuatorName, tibia);
}
