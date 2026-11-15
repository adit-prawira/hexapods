#include "events/journal.hpp"
#include "events/input_event.hpp"
#include <cstdint>
#include <ios>

namespace {
uint64_t hashFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    uint64_t hash = 1469598103934665603ull;
    for (int byte = in.get(); byte != EOF; byte = in.get()) {
        hash ^= static_cast<uint64_t>(byte);
        hash *= 1099511628211ull;
    }

    return hash;
}
} // namespace

void Journal::open(const std::string& path, const std::string& modelPath, double timestep,
                   uint64_t seed) {
    output_.open(path, std::ios::out | std::ios::trunc);
    output_ << "hexapods-journal v1\n";
    output_ << "model " << modelPath << " " << std::hex << hashFile(modelPath) << std::dec << "\n";
    output_ << "timestep " << timestep << "\n";
    output_ << "seed " << seed << "\n";
}

void Journal::append(const InputEvent& event) {
    if (!output_.is_open())
        return;

    output_ << event.stepIndex << " " << static_cast<char>(event.type) << " " << event.payload
            << "\n";
}

void Journal::close() {
    if (!output_.is_open())
        return;
    output_.close();
}
