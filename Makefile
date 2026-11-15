MUJOCO_ROOT ?= $(HOME)/.mujoco/mujoco-4.0.1
BUILD_DIR ?= build

.PHONY: build run test format clean

build:
	cmake -S . -B $(BUILD_DIR) -DMUJOCO_ROOT=$(MUJOCO_ROOT)
	cmake --build $(BUILD_DIR) --parallel

run: build
	$(BUILD_DIR)/hexapods

test: build
	$(BUILD_DIR)/hexapods --scenario smoke
	$(BUILD_DIR)/hexapods --scenario determinism
	@echo "all scenario gates passed"

format:
	clang-format -i $(shell find src tests \( -name '*.cpp' -o -name '*.hpp' \))

clean:
	rm -rf $(BUILD_DIR)
