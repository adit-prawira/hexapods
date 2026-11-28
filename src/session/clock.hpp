#pragma once
#include <cstdint>

class Clock {
public:
  explicit Clock(double timestep) : timestep_(timestep) {}
  void togglePause() { is_paused_ = !is_paused_; }

  bool is_paused() const { return is_paused_; }

  // Feed real elapsed seconds, returning how many stpes are due.
  // Stalls are clampd so a frozen frame can never fast-forwards physics.
  uint64_t advance(double seconds);

  void startEpoch() { unsimulatedTime_ = 0.0; }
  double timestep() const { return timestep_; }

private:
  double timestep_;
  bool is_paused_ = false;

  double unsimulatedTime_ = 0.0;
};
