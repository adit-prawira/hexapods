#include "world/world.hpp"
#include "mujoco/mujoco.h"
#include <algorithm>
#include <cmath>
#include <mujoco/mjtype.h>
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

    // Trunk is the robot central body, and needed to be resolved once. Models without
    // a trunk simply cannot be resolved
    trunkBodyId_ = mj_name2id(model_, mjOBJ_BODY, "trunk");
    if (trunkBodyId_ >= 0)
        startTrunkXPosition_ = data_->xpos[3 * trunkBodyId_];
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
    // Recompute derived quantities (xpos, xquat) so the world is observable
    // immediately after reset — same contract as construction. Without this,
    // the HUD reads one frame of stale pre-reset state after R.
    mj_forward(model_, data_);
    stepIndex_ = 0;
    if (trunkBodyId_ >= 0)
        startTrunkXPosition_ = data_->xpos[3 * trunkBodyId_];
}

void World::enableActuation() { model_->opt.disableflags &= ~mjDSBL_ACTUATION; }

void World::disabledActuation() { model_->opt.disableflags |= mjDSBL_ACTUATION; }

bool World::isActuationEnabled() const { return (model_->opt.disableflags & mjDSBL_ACTUATION) == 0; }

void World::setActuatorTarget(const std::string& actuatorName, double target) {
    const int actuator = mj_name2id(model_, mjOBJ_ACTUATOR, actuatorName.c_str());
    bool is_found = actuator >= 0;
    if (!is_found)
        throw std::runtime_error("no actuatore named '" + actuatorName + "' in model");

    data_->ctrl[actuator] = target;
}

double World::maxJointTrackingError() const {
    double worst = 0.0;
    auto total_actuators = model_->nu;
    for (int actuator = 0; actuator < total_actuators; ++actuator) {
        const int joint = model_->actuator_trnid[2 * actuator];
        const int qpositionAddress = model_->jnt_qposadr[joint];
        const double error = std::abs(data_->qpos[qpositionAddress] - data_->ctrl[actuator]);
        worst = std::max(worst, error);
    }
    return worst;
}

double World::trunkPitch() const {
    checkTrunkRequirement();
    const mjtNum forward[3] = {1, 0, 0};
    mjtNum forwardWorld[3] = {0, 0, 0};
    mju_rotVecQuat(forwardWorld, forward, &data_->xquat[4 * trunkBodyId_]);
    return std::asin(std::clamp(forwardWorld[2], -1.0, 1.0));
}

double World::trunkHeight() const {
    checkTrunkRequirement();
    return data_->xpos[3 * trunkBodyId_ + 2];
}

double World::traveledDistance() const {
    checkTrunkRequirement();
    return data_->xpos[3 * trunkBodyId_] - startTrunkXPosition_;
}

void World::checkTrunkRequirement() const {
    if (trunkBodyId_ < 0)
        throw std::runtime_error("model has no body 'trunk'");
}
