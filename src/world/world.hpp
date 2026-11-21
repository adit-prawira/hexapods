#pragma once
#include <cstdint>
#include <mujoco/mujoco.h>
#include <string>

class World {
  public:
    explicit World(const std::string& modelPath);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    void step();
    void reset();

    const mjModel& model() const { return *model_; }
    const mjData& data() const { return *data_; }
    mjData& renderData() { return *data_; }
    double timestep() const { return model_->opt.timestep; }
    uint64_t stepIndex() const { return stepIndex_; }

    // Apply all forces (mjDSBL_ACTUATION)
    void enableActuation();

    // Removes all forces, so the robot answer only to gravity and contact
    void disabledActuation();

    bool isActuationEnabled() const;

    // Control writes intent by targeting actuator name
    // this will throw exception if actuator name does not exist
    void setActuatorTarget(const std::string& actuatorName, double target);

    // Facts observation about what is physically true currently.
    // The HUD will display them
    double maxJointTrackingError() const; // worst |qpos - ctrl|, radians (value for safety metric and evaluation)
    double trunkPitch() const;            // leg-0-side elevation, radians
    double trunkHeight() const;           // trunk center above the floor, meters
    double traveledDistance() const;      // trunk displacmeent along the +x-axis since spawn/reset (meters)
  private:
    // Throws when model has no trunk. Trunk observation can only be asked
    // of a wolrd that has one
    void checkTrunkRequirement() const;
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    uint64_t stepIndex_ = 0;
    int trunkBodyId_ = -1;
    double startTrunkXPosition_ = 0.0;
};
