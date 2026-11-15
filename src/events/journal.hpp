#pragma once
#include <fstream>
#include <string>

#include "events/input_event.hpp"
class Journal {
  public:
    void open(const std::string& path, const std::string& modelPath, double timestep,
              uint64_t seed);
    void append(const InputEvent& event);
    void close();

  private:
    std::fstream output_;
};
