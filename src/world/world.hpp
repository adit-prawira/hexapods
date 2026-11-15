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

  private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    uint64_t stepIndex_ = 0;
};
