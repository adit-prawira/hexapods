MUJOCO_ROOT ?= $(HOME)/.mujoco/mujoco-4.0.1
BUILD_DIR ?= build

.PHONY: build run test format clean

build:
	cmake -S . -B $(BUILD_DIR) -DMUJOCO_ROOT=$(MUJOCO_ROOT)
	cmake --build $(BUILD_DIR) --parallel

run: build
	$(BUILD_DIR)/hexapods

test: build
	$(BUILD_DIR)/hexapods --model models/scene.xml --scenario smoke
	$(BUILD_DIR)/hexapods --scenario determinism
	$(BUILD_DIR)/hexapods --scenario stand
	$(BUILD_DIR)/hexapods --scenario sanity
	$(BUILD_DIR)/hexapods --scenario walk-5m
	$(BUILD_DIR)/hexapods --scenario throughput
	$(BUILD_DIR)/hexapods --scenario slider-determinism
	$(BUILD_DIR)/hexapods --scenario gait-switch
	$(BUILD_DIR)/hexapods --scenario turn-360
	$(BUILD_DIR)/hexapods --scenario speed-range
	$(BUILD_DIR)/hexapods --scenario sit-stand
	@echo "all scenario gates passed"

format:
	clang-format -i $(shell find src tests \( -name '*.cpp' -o -name '*.hpp' \))

clean:
	rm -rf $(BUILD_DIR)
