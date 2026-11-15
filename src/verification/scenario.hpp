#pragma once
#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

// Verification domain: the scenario harness. Runs named behavioral claims
// headless — no window, no wall clock — and reports pass/fail. This is the
// project's single testing seam (PRD #1).
//
// Separation of concerns: the harness lives here (product code); the gates
// themselves are authored in tests/ and self-register at startup.
namespace verification {

// A gate: runs against the model at modelPath, returns true on PASS.
using ScenarioFunction = bool (*)(const std::string& modelPath, uint64_t seed);

// Registers a gate by name. Called at static-init time from tests/.
void registerScenario(const std::string& name, ScenarioFunction scenario);

// Runs the named gate. Returns true on PASS; false on FAIL or unknown name.
bool runScenario(const std::string& name, const std::string& modelPath, uint64_t seed);

// FNV-1a over the raw bytes of the dynamic state (qpos + qvel) — the
// fingerprint that makes "bit-identical" checkable. Shared by the
// determinism gates and, later, ghost replay validation.
uint64_t hashState(const mjModel& model, const mjData& data);

} // namespace verification
