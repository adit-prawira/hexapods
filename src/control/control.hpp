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
    enum class Pose { Sit, Stand };

    explicit Control(World& world);

    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;

    void holdStandPose();
    void startWalking();
    void sit();
    void stand();
    bool isSitCommanded() const;

    void update();
    GaitParameters& gaitParameters() { return parameters_; }
    void resetGaitParameters() { parameters_ = GaitParameters{}; }

  private:
    void adoptNewGait();
    void setLegTargetAngles(int legId, double coxa, double femur, double tibia);

    double calculateCoxaTargetAngle(int legId, const double& legPhase, const double& dutyFactor) const;
    double calculateFemurTargetAngle(const double& heightScale) const;
    double calculateTibiaTargetAngle(const double& legPhase, const double& heightScale, const double& dutyFactor) const;

    static constexpr int kLegCount = 6;
    static constexpr int kLegJointCount = 3;

    struct PoseTransition {
        Pose destination = Pose::Sit;
        std::array<double, kLegCount * kLegJointCount> startTargets{};
        std::array<double, kLegCount * kLegJointCount> endTargets{};
        int stepsElapsed = 0;
        int stepsTotal = 0;
    };

    void beginPoseTransition(Pose destination);
    void advancePoseTransition();
    void captureCurrentTargets(std::array<double, kLegCount * 3>& targets) const;
    bool isTransitioningTo(Pose target) const;
    bool isInterruptingAnotherTransition() const;

    World& world_;
    GaitParameters parameters_;

    bool isWalking_ = false;
    double gaitPhase_ = 0.0;

    GaitParameters::Gait activeGait_ = GaitParameters::Gait::Tripod;

    bool pendingSit_ = false;
    bool isSitting_ = false;
    std::optional<PoseTransition> poseTransition_;

    std::array<double, kLegCount> sweepSign_;
    std::array<double, kLegCount> sideSign_;
    std::array<double, kLegCount> coxaLimit_;
    std::array<double, kLegCount> footReach_;
};
