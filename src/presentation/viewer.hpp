#pragma once
#include "control/control.hpp"
#include "events/journal.hpp"
#include "presentation/camera.hpp"
#include "session/clock.hpp"
#include "world/world.hpp"
#include <GLFW/glfw3.h>
#include <cstdint>
#include <memory>
#include <mujoco/mujoco.h>

struct WindowState;

class Viewer {
  public:
    Viewer(World& world, Clock& clock, Journal& journal, Control& control);
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    uint64_t run();

  private:
    void handleKeys();
    void updateCamera();
    void stepWorld(double frameSeconds);
    void draw();
    void drawHud();

    World& world_;
    Clock& clock_;
    Journal& journal_;
    Control& control_;
    Camera camera_;
    GLFWwindow* window_ = nullptr;
    std::unique_ptr<WindowState> windowState_;
    mjvOption option_{};
    mjvPerturb perturb_{};
    mjvScene scene_{};
    mjrContext context_{};

    bool pauseKeyWasDown_ = false;
    bool resetKeyWasDown_ = false;
    bool gaitKeyWasDown_ = false;
    bool poseKeyWasDown_ = false;

    double previousMouseX_ = 0.0;
    double previousMouseY_ = 0.0;

    uint64_t totalSteps_ = 0;
};
