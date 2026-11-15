#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "events/journal.hpp"
#include "presentation/viewer.hpp"
#include "session/clock.hpp"
#include "verification/scenario.hpp"
#include "world/world.hpp"

namespace {
const char* kUsage =
    "usage hexapods [--model <path>] [--journal <path>] [--seed <n>] [--scenario <name>]\n"
    " --model     MJCF scene to load (default: models/scene.xml)"
    " --journal   input journal output file (default: journal.log)\n"
    " --seed      RNG seed, recorded in the journal (default: 0)\n"
    " --scenario  headless pass/fail scenario";
};

int main(int argc, char** argv) {
    std::string modelPath = "models/scene.xml";
    std::string journalPath = "journal.log";
    std::string scenario;

    uint64_t seed = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n%s\n", argument.c_str(), kUsage);
                std::exit(2);
            }
            return argv[++i];
        };

        if (argument == "--model")
            modelPath = value();
        else if (argument == "--journal")
            journalPath = value();
        else if (argument == "--seed")
            seed = std::strtoull(value().c_str(), nullptr, 10);
        else if (argument == "--scenario")
            scenario = value();
        else {
            std::fprintf(stderr, "unknown argument %s\n%s", argument.c_str(), kUsage);
            return 2;
        }
    }

    if (!scenario.empty()) {
        // Headless Verification front-end — the one testing seam.
        return verification::runScenario(scenario, modelPath, seed) ? 0 : 1;
    }

    try {
        World world(modelPath);
        Clock clock(world.timestep());
        Journal journal;
        journal.open(journalPath, modelPath, world.timestep(), seed);

        Viewer viewer(world, clock, journal);
        const uint64_t totalSteps = viewer.run();
        journal.close();
        std::printf("session: %llu steps, journal: %s\n",
                    static_cast<unsigned long long>(totalSteps), journalPath.c_str());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
}
