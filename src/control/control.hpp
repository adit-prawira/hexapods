#pragma once
#include "world/world.hpp"

struct GaitParameters {
    enum class Gait { Tripod, Ripple, Wave };

    double frequency = 1.5;     // gait cycles per second
    double stride = 0.10;       // foot sweep per cycle, meters
    double stanceHeight = 0.16; // trunk height while walking, meters
    double speed = 1.0;         // frequency multiplier; 1.0 = nominal
    double turnRate = 0.0;      // rad/s
    Gait gait = Gait::Tripod;   // Ripple/Wave back to Tripod
};

class Control {
  public:
    explicit Control(World& world);

    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;

    void holdStandPose();
    void startWalking();

    void update();
    GaitParameters& parameters() { return parameters_; }

  private:
    void writeLegTargets(int legId, double coxa, double femur, double tibia);

    static constexpr int kLegCount = 6;

    World& world_;
    GaitParameters parameters_;

    bool is_walking_ = false;
    std::array<double, kLegCount> sweepSign_;
    std::array<double, kLegCount> footReach_;
};
