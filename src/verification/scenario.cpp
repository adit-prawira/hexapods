#include "verification/scenario.hpp"

#include <cstdio>
#include <map>
#include <stdexcept>

namespace verification {
namespace {

std::map<std::string, ScenarioFunction>& registry() {
    static std::map<std::string, ScenarioFunction> scenarios;
    return scenarios;
}

} // namespace

void registerScenario(const std::string& name, ScenarioFunction scenario) {
    registry()[name] = scenario;
}

bool runScenario(const std::string& name, const std::string& modelPath, uint64_t seed) {
    const auto& scenarios = registry();
    const auto scenario = scenarios.find(name);
    if (scenario == scenarios.end()) {
        std::printf("unknown scenario '%s' (available:", name.c_str());
        for (const auto& [scenarioName, function] : scenarios) {
            std::printf(" %s", scenarioName.c_str());
        }
        std::printf(")\n");
        return false;
    }
    // A gate that throws (bad model path, missing body) is a FAIL with a
    // message — never an abort that takes the whole test runner down.
    try {
        return scenario->second(modelPath, seed);
    } catch (const std::exception& error) {
        std::printf("scenario '%s': FAIL - %s\n", name.c_str(), error.what());
        return false;
    }
}

uint64_t hashState(const mjModel& model, const mjData& data) {
    uint64_t hash = 1469598103934665603ull;
    const auto mix = [&hash](const mjtNum* values, int count) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(values);
        for (int index = 0; index < count * static_cast<int>(sizeof(mjtNum)); ++index) {
            hash ^= bytes[index];
            hash *= 1099511628211ull;
        }
    };
    mix(data.qpos, model.nq);
    mix(data.qvel, model.nv);
    return hash;
}

} // namespace verification
