#pragma once
#include <mujoco/mujoco.h>

class Camera {
  public:
    static constexpr double kSensitivity = 0.03;

    void frameModel(const mjModel& model) {
        mjv_defaultCamera(&camera_);
        camera_.type = mjCAMERA_FREE;
        for (int axis = 0; axis < 3; ++axis) {
            camera_.lookat[axis] = model.stat.center[axis];
        }
        camera_.distance = 1.6 * model.stat.extent;
        camera_.azimuth = 135;
        camera_.elevation = -25;
    }

    void orbit(const mjModel& model, double dx, double dy) {
        mjv_moveCamera(&model, mjMOUSE_ROTATE_V, kSensitivity * dx, kSensitivity * dy, &camera_);
    }

    void pan(const mjModel& model, double dx, double dy) {
        mjv_moveCamera(&model, mjMOUSE_MOVE_V, kSensitivity * dx, kSensitivity * dy, &camera_);
    }

    void zoom(const mjModel& model, double dx, double dy) {
        mjv_moveCamera(&model, mjMOUSE_ZOOM, kSensitivity * dx, kSensitivity * dy, &camera_);
    }

    void zoomScroll(const mjModel& model, double scroll) {
        mjv_moveCamera(&model, mjMOUSE_ZOOM, 0.0, -0.1 * scroll, &camera_);
    }

    mjvCamera& state() { return camera_; }

  private:
    mjvCamera camera_{};
};
