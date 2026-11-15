// Behavioral gates for the hexapods scenario harness. Each gate is a claim:
// "given [scenario], then [expectation]". Gates self-register with the
// Verification harness at startup; `make test` runs them.
#include <cstdio>

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

// Self-registration: the gates exist in the suite because this initializer
// runs at startup, before main.
const bool registered = [] {
    verification::registerScenario("smoke", &scenarioSmoke);
    verification::registerScenario("determinism", &scenarioDeterminism);
    return true;
}();

} // namespace
