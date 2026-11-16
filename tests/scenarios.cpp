// Behavioral gates for the hexapods scenario harness. Each gate is a claim:
// "given [scenario], then [expectation]". Gates self-register with the
// Verification harness at startup; `make test` runs them.
#include <algorithm>
#include <cstdio>

#include "control/control.hpp"
#include "verification/scenario.hpp"
#include "world/world.hpp"

namespace {

// Given the dev scene and 500 steps of gravity, then drop_box falls from its
// 1.2 m start to below 0.3 m. The threshold comes from physics first
// principles, not from the code: free fall covers the 1.08 m gap in ~0.47 s,
// resting height is 0.12 m — 0.3 m sits far from both.
bool scenarioSmoke(const std::string& modelPath, uint64_t /*seed*/) {
    World world(modelPath);
    const mjModel& model = world.model();
    const int dropBoxBody = mj_name2id(&model, mjOBJ_BODY, "drop_box");
    if (dropBoxBody < 0) {
        std::printf("smoke: FAIL - body 'drop_box' not found in model\n");
        return false;
    }

    const double startZ = world.data().xpos[3 * dropBoxBody + 2];
    constexpr int stepCount = 500; // 1.0 s of simulated time at dt = 0.002
    for (int step = 0; step < stepCount; ++step) {
        world.step();
    }
    const double endZ = world.data().xpos[3 * dropBoxBody + 2];

    const bool passed = endZ < startZ && endZ < 0.3;
    std::printf("smoke: given the dev scene and %d steps of gravity, "
                "then drop_box falls %.3f -> %.3f: %s\n",
                stepCount, startZ, endZ, passed ? "PASS" : "FAIL");
    return passed;
}

// Given two identical worlds stepped in lockstep, then their state streams
// are bit-identical. Two independently constructed Worlds — the comparison
// can never see a value next to itself.
bool scenarioDeterminism(const std::string& modelPath, uint64_t /*seed*/) {
    World firstWorld(modelPath);
    World secondWorld(modelPath);
    constexpr int stepCount = 500;

    for (int step = 0; step < stepCount; ++step) {
        firstWorld.step();
        secondWorld.step();
        if (verification::hashState(firstWorld.model(), firstWorld.data()) !=
            verification::hashState(secondWorld.model(), secondWorld.data())) {
            std::printf("determinism: FAIL - state streams diverged at step %d\n", step);
            return false;
        }
    }

    std::printf("determinism: given two identical worlds stepped %dx in lockstep, "
                "then state streams are bit-identical: PASS\n",
                stepCount);
    return true;
}

// Given the hexapod launched with the stand pose commanded, then it stands
// up and holds: across the final 60 s of a 64 s run (the first 4 s absorbs
// the stand-up transient), the worst joint tracking error stays below
// 0.05 rad and the trunk never sinks below 0.10 m. The thresholds are
// independent of the code: 0.05 rad is the story #10 acceptance number, and
// the height band comes from geometry — standing is 0.16 m (0.32 m of tibia
// drop less 0.16 m of femur rise), belly rest is 0.055 m; 0.10 m sits far
// from both.
bool scenarioStand(const std::string& modelPath, uint64_t /*seed*/) {
    World world(modelPath);
    Control control(world);
    control.holdStandPose();

    constexpr int standUpSteps = 2000; // 4.0 s for the stand-up transient
    constexpr int holdSteps = 30000;   // 60.0 s the story requires held
    for (int step = 0; step < standUpSteps; ++step) {
        world.step();
    }

    double worstTrackingError = 0.0;
    double lowestTrunkHeight = 2.0;
    for (int step = 0; step < holdSteps; ++step) {
        world.step();
        worstTrackingError = std::max(worstTrackingError, world.maxJointTrackingError());
        lowestTrunkHeight = std::min(lowestTrunkHeight, world.trunkHeight());
    }

    const bool passed = worstTrackingError < 0.05 && lowestTrunkHeight >= 0.10;
    std::printf("stand: given the stand pose commanded from launch, then over "
                "the last 60 s the worst tracking error is %.3f rad (need < 0.050) "
                "and the lowest trunk height is %.3f m (need >= 0.100): %s\n",
                worstTrackingError, lowestTrunkHeight, passed ? "PASS" : "FAIL");
    return passed;
}

// Given the hexapod launched with actuation disabled, then gravity wins:
// within 3 s the trunk rests below 0.08 m. Thresholds are independent of
// the code: free fall from the 0.30 m spawn takes ~0.25 s, the belly rest
// is 0.055 m (half the trunk) and the active stand is 0.16 m — 0.08 m sits
// far from both. The permanent honest-physics check: no hidden force can
// hold the robot up.
bool scenarioSanity(const std::string& modelPath, uint64_t /*seed*/) {
    World world(modelPath);
    Control control(world);
    control.holdStandPose(); // targets parked; the breaker stays open
    world.disabledActuation();

    constexpr int collapseSteps = 1500; // 3.0 s of pure gravity and contact
    for (int step = 0; step < collapseSteps; ++step) {
        world.step();
    }

    const double trunkHeight = world.trunkHeight();
    const bool passed = trunkHeight < 0.08;
    std::printf("sanity: given actuation disabled from launch, then after 3.0 s of "
                "gravity the trunk rests at %.3f m (need < 0.080): %s\n",
                trunkHeight, passed ? "PASS" : "FAIL");
    return passed;
}

// Self-registration: the gates exist in the suite because this initializer
// runs at startup, before main.
const bool registered = [] {
    verification::registerScenario("smoke", &scenarioSmoke);
    verification::registerScenario("determinism", &scenarioDeterminism);
    verification::registerScenario("stand", &scenarioStand);
    verification::registerScenario("sanity", &scenarioSanity);
    return true;
}();

} // namespace
