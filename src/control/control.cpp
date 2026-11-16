#include "control/control.hpp"
#include <cstdio>

namespace {
constexpr int kLegCount = 6;

struct JointTarget {
    const char* joint;
    double target;
};

// The neutral stand pose, per joint, in radians. Positive pitch rotates a leg
// segment downward, negative upward: the femur rears up 75 deg, the tibia
// folds down 160 deg to plant the claw — knees high, trunk lifted.
constexpr JointTarget kStandPose[]{{"coxa", 0.0}, {"femur", -1.31}, {"tibia", 2.79}};
} // namespace

Control::Control(World& world) : world_(world) {}

void Control::holdStandPose() {
    for (int legId = 0; legId < kLegCount; ++legId) {
        for (const JointTarget& joint : kStandPose) {
            char actuatorName[32];
            std::snprintf(actuatorName, sizeof(actuatorName), "leg%d_%s", legId, joint.joint);
            world_.setActuatorTarget(actuatorName, joint.target);
        }
    }
}
