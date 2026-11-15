#include "world/world.hpp"
#include "mujoco/mujoco.h"
#include <stdexcept>
#include <string>

World::World(const std::string& modelPath) {
    char error[1024] = "";
    model_ = mj_loadXML(modelPath.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
        throw std::runtime_error("mj_loadXML failed for '" + modelPath + "': " + error);
    }

    data_ = mj_makeData(model_);
    // Compute derived quantities (body positions, etc.) so the world is in a
    // consistent, observable state from construction — before any step.
    mj_forward(model_, data_);
}

World::~World() {
    mj_deleteData(data_);
    mj_deleteModel(model_);
}

void World::step() {
    mj_step(model_, data_);
    ++stepIndex_;
}

void World::reset() {
    mj_resetData(model_, data_);
    stepIndex_ = 0;
}
