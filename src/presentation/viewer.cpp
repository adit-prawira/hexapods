#include "presentation/viewer.hpp"
#include "events/input_event.hpp"
#include "events/journal.hpp"
#include "mujoco/mujoco.h"
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace {
void onGlfwError(int code, const char* description) {
    std::fprintf(stderr, "glfw error %d: %s\n", code, description);
}
} // namespace

struct WindowState {
    double scrollDelta = 0.0;
};

Viewer::Viewer(World& world, Clock& clock, Journal& journal)
    : world_(world), clock_(clock), journal_(journal) {
    glfwSetErrorCallback(onGlfwError);
    if (!glfwInit())
        throw std::runtime_error("glfwInit failed");

    window_ = glfwCreateWindow(1280, 800, "hexapods", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    windowState_ = std::make_unique<WindowState>();
    glfwSetWindowUserPointer(window_, windowState_.get());
    glfwSetScrollCallback(window_, [](GLFWwindow* window, double _deltaX, double deltaY) {
        auto& state = *static_cast<WindowState*>(glfwGetWindowUserPointer(window));
        state.scrollDelta += deltaY;
    });

    mjv_defaultOption(&option_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjv_makeScene(&world_.model(), &scene_, 2000);
    mjr_defaultContext(&context_);
    mjr_makeContext(&world_.model(), &context_, 150);

    camera_.frameModel(world.model());
}

Viewer::~Viewer() {
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

uint64_t Viewer::run() {
    glfwGetCursorPos(window_, &previousMouseX_, &previousMouseY_);
    double lastFrameTime = glfwGetTime();

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        handleKeys();
        updateCamera();

        const double now = glfwGetTime();
        const double frameSeconds = now - lastFrameTime;

        lastFrameTime = now;
        stepWorld(frameSeconds);
        draw();
    }

    return totalSteps_;
}

void Viewer::handleKeys() {
    const bool pauseKeyDown = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (pauseKeyDown && !pauseKeyWasDown_)
        clock_.togglePause();

    pauseKeyWasDown_ = pauseKeyDown;

    const bool resetKeyDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
    if (resetKeyDown && !resetKeyWasDown_) {
        world_.reset();
        clock_.startEpoch();
        journal_.append({0, InputEvent::Type::Reset, "reset"});
    }

    resetKeyWasDown_ = resetKeyDown;
}

void Viewer::updateCamera() {
    double mouseX = 0.0;
    double mouseY = 0.0;

    glfwGetCursorPos(window_, &mouseX, &mouseY);

    const double dragX = mouseX - previousMouseX_;
    const double dragY = mouseY - previousMouseY_;

    previousMouseX_ = mouseX;
    previousMouseY_ = mouseY;

    const mjModel& model = world_.model();

    if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        camera_.orbit(model, dragX, dragY);
    }

    if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        camera_.pan(model, dragX, dragY);
    }

    if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        camera_.zoom(model, dragX, dragY);
    }

    if (windowState_->scrollDelta != 0.0) {
        camera_.zoomScroll(model, windowState_->scrollDelta);
        windowState_->scrollDelta = 0.0;
    }
};

void Viewer::stepWorld(double frameSeconds) {
    const uint64_t stepsDue = clock_.advance(frameSeconds);
    for (uint64_t step = 0; step < stepsDue; ++step) {
        world_.step();
        ++totalSteps_;
    }
}

void Viewer::draw() {
    int frameBufferWidth = 0;
    int frameBufferHeight = 0;

    glfwGetFramebufferSize(window_, &frameBufferWidth, &frameBufferHeight);
    mjv_updateScene(&world_.model(), &world_.renderData(), &option_, &perturb_, &camera_.state(),
                    mjCAT_ALL, &scene_);
    mjr_render(mjrRect{0, 0, frameBufferWidth, frameBufferHeight}, &scene_, &context_);
    glfwSwapBuffers(window_);
}
