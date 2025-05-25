# FPGA Build Makefile

# Build directory
BUILD_DIR = _build

# Default target
all: $(BUILD_DIR)/hardware.bit

# Create build directory and generate file list
$(BUILD_DIR)/croc.flist: | $(BUILD_DIR)
	bender script flist-plus -t fpga -t rtl --define SYNTHESIS=1 -t ecp5 > $@

# Run Yosys synthesis
$(BUILD_DIR)/hardware.json: $(BUILD_DIR)/croc.flist
	yosys -s run_yosys.ys

# Run place and route with nextpnr
$(BUILD_DIR)/hardware.config: $(BUILD_DIR)/hardware.json
	nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json $(BUILD_DIR)/hardware.json --textcfg $(BUILD_DIR)/hardware.config --report $(BUILD_DIR)/hardware.pnr --lpf constraints/iCESugar-Pro.lpf

# Generate bitstream and SVF file
$(BUILD_DIR)/hardware.bit: $(BUILD_DIR)/hardware.config
	ecppack --svf $(BUILD_DIR)/hardware.svf $(BUILD_DIR)/hardware.config $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Clean target - removes build directory
clean:
	rm -rf $(BUILD_DIR)

# Phony targets
.PHONY: all clean