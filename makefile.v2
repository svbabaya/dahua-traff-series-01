# Makefile ver.2 add separate build directory
# TraffiXtream build for DHOP 
# How to use: make [ all | clean | strip | debug | release | info ]

CXX = arm-himix200-linux-v2-g++

PROGS = TraffiXtreamS

COMM = common
COMM_SERV = common/server
COMM_TRAF = common/traffix
COMM_OPENCV_I = common/opencv/include
COMM_FEATUREDET = common/featureTracker
COMM_SEGMENTDET = common/segmentTracker

COMM_OPENCV_L = common/opencv/arm/lib
COMM_OPENCV_LE = common/opencv/arm/3rdparty/lib

ARCH_FLAGS = -mcpu=cortex-a7 -mfloat-abi=hard -mfpu=neon-vfpv4

# Flags for optimization
DEBUG_OPTIM = -O0 -g -DDEBUG
RELEASE_OPTIM = -O2 -fomit-frame-pointer -ffast-math -ftree-vectorize -DNDEBUG

# Base flags
BASE_CFLAGS = $(ARCH_FLAGS) -Wall -Wextra -Wno-unknown-pragmas
BASE_CFLAGS += -Isource -I$(COMM) -I$(COMM_SERV) -I$(COMM_TRAF) -I$(COMM_OPENCV_I) -I$(COMM_FEATUREDET) -I$(COMM_SEGMENTDET)

# Build directories
BUILD_DIR = build
RELEASE_DIR = $(BUILD_DIR)/release
DEBUG_DIR = $(BUILD_DIR)/debug

# Variable for mode choice (debug or release)
BUILD_TYPE ?= release

# Select build directory based on type
ifeq ($(BUILD_TYPE),debug)
    BUILD_OUTPUT_DIR = $(DEBUG_DIR)
    OPTIM_FLAGS = $(DEBUG_OPTIM)
    STRIP_CMD = @echo "Debug build - skipping strip"
else
    BUILD_OUTPUT_DIR = $(RELEASE_DIR)
    OPTIM_FLAGS = $(RELEASE_OPTIM)
    STRIP_CMD = arm-himix200-linux-v2-strip --strip-unneeded $(BUILD_OUTPUT_DIR)/$(PROGS)
endif

CFLAGS = $(BASE_CFLAGS) $(OPTIM_FLAGS)
CXXFLAGS = $(CFLAGS) -std=c++11 -fno-rtti -fexceptions

LDFLAGS += $(ARCH_FLAGS) -lpthread -lrt

LOPENCV = -L$(COMM_OPENCV_L) -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_highgui -lopencv_core -L$(COMM_OPENCV_LE) -llibjpeg -lzlib -lrt -lpthread -lm -ldl -lstdc++

SRCS = source/main.cpp $(COMM)/signalhandler.cpp $(COMM)/capturehandler.cpp $(COMM)/globals_cam.cpp $(COMM)/datastructs.cpp $(COMM)/utils_cam.cpp $(COMM_SERV)/tcpServerSource.cpp $(COMM_SERV)/tcpServerClientHandle.cpp $(COMM_SERV)/tcpServerResponse.cpp $(COMM_SERV)/tcpServerResponse_traff.cpp $(COMM_TRAF)/utils_traff.cpp $(COMM_TRAF)/traffzone.cpp $(COMM_TRAF)/traffsensor.cpp $(COMM_TRAF)/traffcounter.cpp $(COMM_FEATUREDET)/featureTracker.cpp $(COMM_FEATUREDET)/featureReg.cpp $(COMM_SEGMENTDET)/segmentTracker.cpp

# Object files will be placed in build directory with same relative paths
OBJS = $(patsubst %.cpp,$(BUILD_OUTPUT_DIR)/%.o,$(SRCS))

# Create build directory structure
$(shell mkdir -p $(RELEASE_DIR)/source $(RELEASE_DIR)/$(COMM) $(RELEASE_DIR)/$(COMM_SERV) $(RELEASE_DIR)/$(COMM_TRAF) $(RELEASE_DIR)/$(COMM_FEATUREDET) $(RELEASE_DIR)/$(COMM_SEGMENTDET))
$(shell mkdir -p $(DEBUG_DIR)/source $(DEBUG_DIR)/$(COMM) $(DEBUG_DIR)/$(COMM_SERV) $(DEBUG_DIR)/$(COMM_TRAF) $(DEBUG_DIR)/$(COMM_FEATUREDET) $(DEBUG_DIR)/$(COMM_SEGMENTDET))

# Pattern rule to compile .cpp files to .o files in build directory
$(BUILD_OUTPUT_DIR)/%.o: %.cpp
	@echo "Compiling $<..."
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Main target
all: $(BUILD_OUTPUT_DIR)/$(PROGS)
	$(STRIP_CMD)
	@echo "Build completed: $(BUILD_OUTPUT_DIR)/$(PROGS)"

# Link executable
$(BUILD_OUTPUT_DIR)/$(PROGS): $(OBJS)
	@echo "Linking $@..."
	$(CXX) $(LDFLAGS) $^ $(LIBS) $(LDLIBS) $(LOPENCV) -o $@
	@echo "Binary size:"
	@ls -lh $@

# Debug mode
debug:
	@echo "Building DEBUG version..."
	$(MAKE) BUILD_TYPE=debug

# Release mode
release:
	@echo "Building RELEASE version..."
	$(MAKE) BUILD_TYPE=release

# Quick strip of existing binaries
strip:
	@if [ -f "$(RELEASE_DIR)/$(PROGS)" ]; then \
		echo "Stripping release version..."; \
		arm-himix200-linux-v2-strip --strip-unneeded $(RELEASE_DIR)/$(PROGS); \
		ls -lh $(RELEASE_DIR)/$(PROGS); \
	fi
	@if [ -f "$(DEBUG_DIR)/$(PROGS)" ]; then \
		echo "Note: Debug version not stripped (contains debug info)"; \
	fi

# Clean all build artifacts
clean:
	@echo "Cleaning build directories..."
	rm -rf $(BUILD_DIR)
	rm -f *.tar
	@echo "Clean completed."

# Show info about builds
info:
	@echo "Build configuration:"
	@echo "  Build type: $(BUILD_TYPE)"
	@echo "  Output dir: $(BUILD_OUTPUT_DIR)"
	@echo "  CXX: $(CXX)"
	@echo "  CFLAGS: $(CFLAGS)"
	@echo ""
	@echo "Available binaries:"
	@if [ -f "$(RELEASE_DIR)/$(PROGS)" ]; then \
		echo "  Release: $(RELEASE_DIR)/$(PROGS)"; \
		ls -lh $(RELEASE_DIR)/$(PROGS); \
	fi
	@if [ -f "$(DEBUG_DIR)/$(PROGS)" ]; then \
		echo "  Debug: $(DEBUG_DIR)/$(PROGS)"; \
		ls -lh $(DEBUG_DIR)/$(PROGS); \
	fi

.PHONY: all clean strip debug release info
