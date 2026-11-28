#include "control/control.hpp"
#include "events/input_event.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

constexpr double kSitCoxa = 0.0;
constexpr double kSitFemur = -0.45;
constexpr double kSitTibia = 0.84;
constexpr JointTarget kSitPose[]{{"coxa", kSitCoxa}, {"femur", kSitFemur}, {"tibia", kSitTibia}};
constexpr double kPoseTransitionSeconds = 1.0;

constexpr double kSwingLift = 0.6;           // lower-lweg fold at mid-swing (radians)
constexpr double kNominalStandHeight = 0.16; // trunk height of the sxtand pose

constexpr double kGaitOffset[3][6] = {{0.0, 0.5, 0.0, 0.5, 0.0, 0.5}, {0.0, 1.0 / 3, 2.0 / 3, 2.0 / 3, 1.0 / 3, 0.0}, {0.0, 1.0 / 6, 2.0 / 6, 3.0 / 6, 4.0 / 6, 5.0 / 6}};

constexpr double kGaitDuty[3] = {0.5, 2.0 / 3, 5.0 / 6};
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
        std::snprintf(footName, sizeof(footName), "leg%d_foot", legId);
        const int footGeometry = mj_name2id(&world_.model(), mjOBJ_GEOM, footName);

        bool is_missing_body_component = legBody < 0 || femurBody < 0 || tibiaBody < 0 || footGeometry < 0;
        if (is_missing_body_component)
            throw std::runtime_error("model is missing some leg bodies or feet component");

        char coxaActuatorName[32];
        std::snprintf(coxaActuatorName, sizeof(coxaActuatorName), "leg%d_coxa", legId);
        const int coxaActuator = mj_name2id(&world_.model(), mjOBJ_ACTUATOR, coxaActuatorName);
        if (coxaActuator < 0)
            throw std::runtime_error("model is missing a coxa actuator");
        coxaLimit_[legId] = world_.model().actuator_ctrlrange[2 * coxaActuator + 1];
        sideSign_[legId] = world_.model().body_pos[3 * legBody + 1] > 0.0 ? 1.0 : -1.0;

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
    isWalking_ = false;
    pendingSit_ = false;
    isSitting_ = false;
    poseTransition_.reset();

    for (int legId = 0; legId < kLegCount; ++legId) {
        for (const JointTarget& joint : kStandPose) {
            char actuatorName[32];
            std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_%s", legId, joint.joint);
            world_.setActuatorTarget(actuatorName, joint.target);
        }
    }
}

void Control::startWalking() {
    isWalking_ = true;
    gaitPhase_ = 0.0;
    activeGait_ = parameters_.gait;
    pendingSit_ = false;
    isSitting_ = false;
    poseTransition_.reset();
}

void Control::sit() {
    if (isSitCommanded())
        return;

    if (isInterruptingAnotherTransition()) {
        beginPoseTransition(Pose::Sit);
        return;
    }

    if (isWalking_) {
        pendingSit_ = true;
        return;
    }
    beginPoseTransition(Pose::Sit);
}

void Control::stand() {
    if (pendingSit_) {
        pendingSit_ = false;
        return;
    }

    if (isTransitioningTo(Pose::Stand))
        return;
    if (isInterruptingAnotherTransition()) {
        beginPoseTransition(Pose::Stand);
        return;
    }
    if (!isSitting_)
        return;
    beginPoseTransition(Pose::Stand);
}

bool Control::isSitCommanded() const { return pendingSit_ || isSitting_ || isTransitioningTo(Pose::Sit); }

void Control::update() {
    if (poseTransition_) {
        advancePoseTransition();
        return;
    }

    if (!isWalking_)
        return;

    // Gait phase is Control's own clock accumulated one step at time
    // thus live frequency change bends the trajectory from where it is instead of teleporting it by (new frequency x elapsed time)
    const double cycles = gaitPhase_;
    const double heightScale = parameters_.stanceHeight / kNominalStandHeight;
    const int gaitIndex = static_cast<int>(activeGait_);
    const double dutyFactor = kGaitDuty[gaitIndex];

    for (int legId = 0; legId < kLegCount; ++legId) {
        const double legPhase = std::fmod(cycles + kGaitOffset[gaitIndex][legId], 1.0);

        const double coxaTargetAngle = calculateCoxaTargetAngle(legId, legPhase, dutyFactor);
        const double femurTargetAngle = calculateFemurTargetAngle(heightScale);
        const double tibiaTargetAngle = calculateTibiaTargetAngle(legPhase, heightScale, dutyFactor);

        setLegTargetAngles(legId, coxaTargetAngle, femurTargetAngle, tibiaTargetAngle);
    }
    const double previousGaitPhase = gaitPhase_;
    gaitPhase_ += world_.timestep() * parameters_.frequency * parameters_.speed;

    const bool justFinishedFullGaitCycle = std::floor(gaitPhase_) > std::floor(previousGaitPhase);
    const bool hasNewGaitToSwitch = parameters_.gait != activeGait_;
    const bool shouldAdoptNewGait = justFinishedFullGaitCycle && hasNewGaitToSwitch;

    if (shouldAdoptNewGait) {
        adoptNewGait();
    }

    if (pendingSit_ && justFinishedFullGaitCycle) {
        beginPoseTransition(Pose::Sit);
    }
}

bool Control::isTransitioningTo(Pose target) const { return poseTransition_.has_value() && poseTransition_->destination == target; }

bool Control::isInterruptingAnotherTransition() const { return poseTransition_.has_value(); }

void Control::beginPoseTransition(Pose destination) {
    pendingSit_ = false;
    const JointTarget* destinationPose = (destination == Pose::Sit) ? kSitPose : kStandPose;
    PoseTransition transition;
    transition.destination = destination;
    transition.stepsTotal = static_cast<int>(kPoseTransitionSeconds / world_.timestep());
    captureCurrentTargets(transition.startTargets);

    for (int legId = 0; legId < kLegCount; ++legId) {
        for (int jointIndex = 0; jointIndex < kLegJointCount; ++jointIndex) {
            transition.endTargets[legId * 3 + jointIndex] = destinationPose[jointIndex].target;
        }
    }
    isWalking_ = false;
    isSitting_ = false;
    poseTransition_ = transition;
}

void Control::advancePoseTransition() {
    PoseTransition& transition = *poseTransition_;
    ++transition.stepsElapsed;

    const double timeFraction = static_cast<double>(transition.stepsElapsed) / transition.stepsTotal;

    // smoothstep polynomial ->  3t² − 2t³ = t²(3 − 2t)
    const double poseBlendFraction = timeFraction * timeFraction * (3.0 - 2.0 * timeFraction);
    for (int legId = 0; legId < kLegCount; ++legId) {
        const auto interpolate = [&](int jointIndex) {
            const int index = legId * kLegJointCount + jointIndex;
            return transition.startTargets[index] + (transition.endTargets[index] - transition.startTargets[index]) * poseBlendFraction;
        };
        setLegTargetAngles(legId, interpolate(0), interpolate(1), interpolate(2));
    }

    const bool hasFinishedPoseTransition = transition.stepsElapsed >= transition.stepsTotal;
    if (hasFinishedPoseTransition) {
        const Pose destination = transition.destination;
        poseTransition_.reset();
        if (destination == Pose::Stand)
            startWalking();
        else
            isSitting_ = true;
    }
}

void Control::captureCurrentTargets(std::array<double, kLegCount * kLegJointCount>& targets) const {
    for (int legId = 0; legId < kLegCount; ++legId) {
        for (int jointIndex = 0; jointIndex < kLegJointCount; ++jointIndex) {
            char actuatorName[32];
            std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_%s", legId, kStandPose[jointIndex].joint);
            const int actuatorId = mj_name2id(&world_.model(), mjOBJ_ACTUATOR, actuatorName);
            targets[legId * kLegJointCount + jointIndex] = world_.data().ctrl[actuatorId];
        }
    }
}

void Control::adoptNewGait() {
    activeGait_ = parameters_.gait;
    gaitPhase_ = 0.0;
}

double Control::calculateCoxaTargetAngle(int legId, const double& legPhase, const double& dutyFactor) const {
    const double turnStride = footReach_[legId] * parameters_.turnRate / (parameters_.frequency * parameters_.speed);
    const double effectiveStride = parameters_.stride + sideSign_[legId] * turnStride;
    const double strideLimit = 2 * footReach_[legId] * std::sin(coxaLimit_[legId]);
    const double peakCoxaAngleForStride = std::asin(std::clamp(effectiveStride, -strideLimit, strideLimit) / (2 * footReach_[legId]));
    const double coxaSweep = sweepSign_[legId] * peakCoxaAngleForStride;

    if (legPhase < dutyFactor) {
        const double stanceProgress = legPhase / dutyFactor;
        return coxaSweep * std::cos(std::numbers::pi * stanceProgress);
    };
    const double swingProgress = (legPhase - dutyFactor) / (1.0 - dutyFactor);
    return -coxaSweep * std::cos(std::numbers::pi * swingProgress);
}

double Control::calculateFemurTargetAngle(const double& heightScale) const { return kStandFemur * heightScale; }

double Control::calculateTibiaTargetAngle(const double& legPhase, const double& heightScale, const double& dutyFactor) const {
    const double tibia = kStandTibia * heightScale;
    if (legPhase < dutyFactor)
        return tibia;

    const double swingProgress = (legPhase - dutyFactor) / (1.0 - dutyFactor);
    return tibia - kSwingLift * std::sin(std::numbers::pi * swingProgress);
}

void Control::setLegTargetAngles(int legId, double coxa, double femur, double tibia) {
    char actuatorName[32];
    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_coxa", legId);
    world_.setActuatorTarget(actuatorName, coxa);

    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_femur", legId);
    world_.setActuatorTarget(actuatorName, femur);

    std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_tibia", legId);
    world_.setActuatorTarget(actuatorName, tibia);
}
