// Behavioral gates for the hexapods scenario harness. Each gate is a claim:
// "given [scenario], then [expectation]". Gates self-register with the
// Verification harness at startup; `make test` runs them.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "control/control.hpp"
#include "events/input_event.hpp"
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

// Given two identical worlds walking in lockstep, then their state streams
// are bit-identical. Two independently constructed Worlds, gait active from
// launch — the comparison can never see a value next to itself, and the gait
// engine's ctrl-writing path is inside the claim.
bool scenarioDeterminism(const std::string& modelPath, uint64_t /*seed*/) {
    World firstWorld(modelPath);
    World secondWorld(modelPath);
    Control firstControl(firstWorld);
    Control secondControl(secondWorld);
    firstControl.startWalking();
    secondControl.startWalking();
    constexpr int stepCount = 500;

    for (int step = 0; step < stepCount; ++step) {
        firstControl.update();
        secondControl.update();
        firstWorld.step();
        secondWorld.step();
        if (verification::hashState(firstWorld.model(), firstWorld.data()) != verification::hashState(secondWorld.model(), secondWorld.data())) {
            std::printf("determinism: FAIL - state streams diverged at step %d\n", step);
            return false;
        }
    }

    std::printf("determinism: given two identical worlds walking %dx in lockstep, "
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

// Given the hexapod standing, then commanded to walk, it covers 5 m of
// forward displacement within 60 s of walking, straight (lateral drift
// <= 0.25 m) with the trunk never near the ground (>= 0.10 m — the stand
// gate's geometric band: standing 0.16 m, belly rest 0.055 m). The 4 s
// stand-up transient is excluded from the clock, as in the stand gate.
// Thresholds are the story #11 acceptance numbers and geometry, not code
// outputs. Stops early at 5 m so the time-to-distance is the honest number.
bool scenarioWalk5m(const std::string& modelPath, uint64_t /*seed*/) {
    World world(modelPath);
    Control control(world);
    control.holdStandPose();

    constexpr int standUpSteps = 2000; // 4.0 s for the stand-up transient
    for (int step = 0; step < standUpSteps; ++step) {
        world.step();
    }

    const int trunkBody = mj_name2id(&world.model(), mjOBJ_BODY, "trunk");
    const double startLateral = world.data().xpos[3 * trunkBody + 1];
    const double startDistance = world.traveledDistance();

    control.startWalking();

    constexpr int walkBudgetSteps = 30000; // 60.0 s at dt = 0.002
    constexpr double targetDistance = 5.0;
    constexpr double maxLateralDrift = 0.25;
    constexpr double minTrunkHeight = 0.10;

    double distanceCovered = 0.0;
    double worstLateralDrift = 0.0;
    double lowestTrunkHeight = 2.0;
    int stepsWalked = 0;

    const auto walkStartWall = std::chrono::steady_clock::now();
    for (int step = 0; step < walkBudgetSteps; ++step) {
        control.update();
        world.step();
        ++stepsWalked;

        distanceCovered = world.traveledDistance() - startDistance;
        worstLateralDrift = std::max(worstLateralDrift, std::abs(world.data().xpos[3 * trunkBody + 1] - startLateral));
        lowestTrunkHeight = std::min(lowestTrunkHeight, world.trunkHeight());

        if (distanceCovered >= targetDistance)
            break;
    }
    const double walkWallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - walkStartWall).count();

    const bool passed = distanceCovered >= targetDistance && worstLateralDrift <= maxLateralDrift && lowestTrunkHeight >= minTrunkHeight;
    std::printf("walk-5m: given the walk commanded after stand-up, then %.2f m covered "
                "in %.1f s (need >= 5.0 m in <= 60.0 s), lateral drift %.3f m "
                "(need <= 0.250), lowest trunk %.3f m (need >= 0.100), "
                "%.0f steps/s: %s\n",
                distanceCovered, stepsWalked * world.timestep(), worstLateralDrift, lowestTrunkHeight, stepsWalked / walkWallSeconds, passed ? "PASS" : "FAIL");
    return passed;
}

// Given the hexapod walking headless, then physics steps at >= 2000 steps/s
// (the PRD performance budget for ~30 DOF). The floor sits far below what the
// model actually achieves, so machine variance cannot flip the verdict.
bool scenarioThroughput(const std::string& modelPath, uint64_t /*seed*/) {
    World world(modelPath);
    Control control(world);
    control.startWalking();

    constexpr int measuredSteps = 5000; // 10.0 s of simulated time
    const auto startWall = std::chrono::steady_clock::now();
    for (int step = 0; step < measuredSteps; ++step) {
        control.update();
        world.step();
    }
    const double wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - startWall).count();
    const double stepsPerSecond = measuredSteps / wallSeconds;

    constexpr double minStepsPerSecond = 2000.0;
    const bool passed = stepsPerSecond >= minStepsPerSecond;
    std::printf("throughput: given %d headless walking steps on the hexapod, then "
                "%.0f steps/s (need >= 2000): %s\n",
                measuredSteps, stepsPerSecond, passed ? "PASS" : "FAIL");
    return passed;
}

// Given the same scripted slider session — InputEvents stamped by effect-step,
// payloads in the exact "key value" format the viewer journals — applied into
// two independently constructed worlds walking in lockstep, then their state
// streams are bit-identical. A third, no-slider baseline must end at a
// DIFFERENT hash: that leg kills the false positive where ignored events leave
// both worlds identical (a pure lockstep gate cannot tell the sliders did
// nothing). Values sit inside the slider ranges; same-stamp events exercise
// last-wins. No threshold is recomputed from code — the claim is bit equality.
bool scenarioSliderDeterminism(const std::string& modelPath, uint64_t /*seed*/) {
    const std::vector<InputEvent> sliderSession{
        {100, InputEvent::Type::Slider, "frequency 2.5"}, {100, InputEvent::Type::Slider, "stride 0.14"}, {300, InputEvent::Type::Slider, "stanceHeight 0.12"},
        {500, InputEvent::Type::Slider, "frequency 1.0"}, {700, InputEvent::Type::Slider, "stride 0.05"}, {700, InputEvent::Type::Slider, "frequency 2.0"},
    };
    constexpr int stepCount = 1000;

    World firstWorld(modelPath);
    World secondWorld(modelPath);
    World baselineWorld(modelPath);
    Control firstControl(firstWorld);
    Control secondControl(secondWorld);
    Control baselineControl(baselineWorld);
    firstControl.startWalking();
    secondControl.startWalking();
    baselineControl.startWalking();

    // Apply the session at the effect-step: when stepIndex() == stamp, before
    // the update() that consumes it — the same convention the viewer stamps by.
    const auto applySession = [&](Control& control, int step) {
        for (const InputEvent& event : sliderSession) {
            if (event.stepIndex != static_cast<uint64_t>(step))
                continue;
            const auto separator = event.payload.find(' ');
            const std::string key = event.payload.substr(0, separator);
            const double value = std::stod(event.payload.substr(separator + 1));
            GaitParameters& parameters = control.gaitParameters();
            if (key == "frequency")
                parameters.frequency = value;
            else if (key == "stride")
                parameters.stride = value;
            else if (key == "stanceHeight")
                parameters.stanceHeight = value;
        }
    };

    for (int step = 0; step < stepCount; ++step) {
        applySession(firstControl, step);
        applySession(secondControl, step);
        firstControl.update();
        secondControl.update();
        baselineControl.update();
        firstWorld.step();
        secondWorld.step();
        baselineWorld.step();
        if (verification::hashState(firstWorld.model(), firstWorld.data()) != verification::hashState(secondWorld.model(), secondWorld.data())) {
            std::printf("slider-determinism: FAIL - state streams diverged at step %d\n", step);
            return false;
        }
    }

    const uint64_t sessionHash = verification::hashState(firstWorld.model(), firstWorld.data());
    const uint64_t baselineHash = verification::hashState(baselineWorld.model(), baselineWorld.data());
    if (sessionHash == baselineHash) {
        std::printf("slider-determinism: FAIL - slider session left the trajectory unchanged (events ignored?)\n");
        return false;
    }

    std::printf("slider-determinism: given the same scripted slider session applied at effect-steps "
                "into two worlds walking %dx in lockstep, then state streams are bit-identical "
                "and differ from the no-slider baseline: PASS\n",
                stepCount);
    return true;
}

// Self-registration: the gates exist in the suite because this initializer
// runs at startup, before main.
const bool registered = [] {
    verification::registerScenario("smoke", &scenarioSmoke);
    verification::registerScenario("determinism", &scenarioDeterminism);
    verification::registerScenario("stand", &scenarioStand);
    verification::registerScenario("sanity", &scenarioSanity);
    verification::registerScenario("walk-5m", &scenarioWalk5m);
    verification::registerScenario("throughput", &scenarioThroughput);
    verification::registerScenario("slider-determinism", &scenarioSliderDeterminism);
    return true;
}();

} // namespace
