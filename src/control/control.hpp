#pragma once
#include "world/world.hpp"

class Control {
  public:
    explicit Control(World& world);

    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;

    void holdStandPose();

  private:
    World& world_;
};
