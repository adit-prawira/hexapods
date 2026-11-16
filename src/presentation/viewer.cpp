#include "presentation/viewer.hpp"
#include "events/input_event.hpp"
#include "events/journal.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "mujoco/mujoco.h"
#include <cmath>
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

Viewer::Viewer(World& world, Clock& clock, Journal& journal, Control& control)
    : world_(world), clock_(clock), journal_(journal), control_(control) {
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

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window_, true); // ImGui chains existing glfw callbacks
    ImGui_ImplOpenGL2_Init();
}

Viewer::~Viewer() {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

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

        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL2_NewFrame();
        ImGui::NewFrame();

        handleKeys();
        updateCamera();

        const double now = glfwGetTime();
        const double frameSeconds = now - lastFrameTime;

        lastFrameTime = now;
        stepWorld(frameSeconds);
        drawHud();
        ImGui::Render();
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
        control_.holdStandPose();
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

    if (ImGui::GetIO().WantCaptureMouse) {
        windowState_->scrollDelta = 0.0;
        return;
    }

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
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

void Viewer::drawHud() {
    const ImGuiIO& io = ImGui::GetIO();
    constexpr double kRadiansToDegree = 180 / M_PI;

    ImGui::SetNextWindowPos(ImVec2(14.0f, 14.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("hexapods", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("fps %.0f", io.Framerate);
    ImGui::Text("steps %llu", static_cast<unsigned long long>(world_.stepIndex()));
    ImGui::Text("tracking error: %.3f rad", world_.maxJointTrackingError());
    ImGui::Text("trunk pitch: %+.1f deg", world_.trunkPitch() * kRadiansToDegree);
    ImGui::Text("trunk height: %.3f", world_.trunkHeight());

    bool isActuatorEnabled = world_.isActuationEnabled();
    if (ImGui::Checkbox("Enable Actuator", &isActuatorEnabled)) {
        if (isActuatorEnabled)
            world_.enableActuation();
        else
            world_.disabledActuation();
        std::string label = isActuatorEnabled ? "Actuator ON" : "Actuator OFF";
        journal_.append({world_.stepIndex(), InputEvent::Type::Toggle, label});
    }
    ImGui::End();
}
