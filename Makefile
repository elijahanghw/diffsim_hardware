CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -MMD -MP $(shell pkg-config --cflags opencv4 realsense2)
LDFLAGS := $(shell pkg-config --libs opencv4 realsense2)

CC := gcc
# Invalid/0 depth reads as max range (far), not the generated default of near/blind-zone --
# matches this project's depth pipeline convention (see src/main.cpp).
CFLAGS := -std=c11 -O2 -Wall -MMD -MP -DCNN_INVALID_IS_FAR=1

PROJECT_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR := $(PROJECT_DIR)src
INCLUDE_DIR := $(PROJECT_DIR)include
BUILD_DIR ?= $(PROJECT_DIR)build

CXXFLAGS += -I$(INCLUDE_DIR)

# USE_VO=1 builds in visual odometry (make USE_VO=1); default is without it,
# which excludes visual_odometry.cpp and the code that uses it.
USE_VO ?= 0

# NO_DISPLAY=1 strips out the cv::imshow/waitKey display window entirely, for
# headless builds (e.g. running on a robot with no GUI backend installed).
# Even without this flag, the display is auto-skipped at runtime when no
# DISPLAY env var is set (plain ssh without X forwarding).
NO_DISPLAY ?= 0

TARGET := $(BUILD_DIR)/depthcam
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
ifeq ($(USE_VO),1)
CXXFLAGS += -DUSE_VO
else
SRCS := $(filter-out $(SRC_DIR)/visual_odometry.cpp,$(SRCS))
endif
ifeq ($(NO_DISPLAY),1)
CXXFLAGS += -DNO_DISPLAY
endif

# USE_RELAY=1 builds in the FC serial bridge (make USE_RELAY=1): relays
# optitrack/setpoint/keyboard UDP to the indiflight flight controller and
# transmits CNN features as NN_INPUT_CHUNK over pi-protocol. Pulls in src/relay/
# and the vendored generated pi-protocol in src/pi_protocol/. Default is without
# it, which excludes both from the build.
USE_RELAY ?= 0

OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

CSRCS := $(wildcard $(SRC_DIR)/cnn/*.c)
COBJS := $(patsubst $(SRC_DIR)/cnn/%.c,$(BUILD_DIR)/cnn/%.o,$(CSRCS))
CDEPS := $(COBJS:.o=.d)

# Relay + pi-protocol (only when USE_RELAY=1; empty otherwise so the link and
# clean lines below stay valid either way).
RELAY_OBJS :=
PI_OBJS :=
RELAY_DEPS :=
PI_DEPS :=
ifeq ($(USE_RELAY),1)
CXXFLAGS += -DUSE_RELAY -I$(SRC_DIR)/pi_protocol -DPI_STATS -DPI_USE_PRINT_MSGS -pthread
LDFLAGS += -pthread
# Generated pi-protocol C is compiled with the C compiler and its feature flags.
PI_CFLAGS := -std=c11 -O2 -Wall -MMD -MP -I$(SRC_DIR)/pi_protocol -DPI_STATS -DPI_USE_PRINT_MSGS
RELAY_SRCS := $(wildcard $(SRC_DIR)/relay/*.cpp)
RELAY_OBJS := $(patsubst $(SRC_DIR)/relay/%.cpp,$(BUILD_DIR)/relay/%.o,$(RELAY_SRCS))
RELAY_DEPS := $(RELAY_OBJS:.o=.d)
PI_SRCS := $(wildcard $(SRC_DIR)/pi_protocol/*.c)
PI_OBJS := $(patsubst $(SRC_DIR)/pi_protocol/%.c,$(BUILD_DIR)/pi_protocol/%.o,$(PI_SRCS))
PI_DEPS := $(PI_OBJS:.o=.d)
endif

.PHONY: all clean regen-pi

all: $(TARGET)

$(TARGET): $(OBJS) $(COBJS) $(RELAY_OBJS) $(PI_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(COBJS) $(RELAY_OBJS) $(PI_OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/cnn/%.o: $(SRC_DIR)/cnn/%.c | $(BUILD_DIR)/cnn
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/relay/%.o: $(SRC_DIR)/relay/%.cpp | $(BUILD_DIR)/relay
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/pi_protocol/%.o: $(SRC_DIR)/pi_protocol/%.c | $(BUILD_DIR)/pi_protocol
	$(CC) $(PI_CFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/cnn:
	mkdir -p $(BUILD_DIR)/cnn

$(BUILD_DIR)/relay:
	mkdir -p $(BUILD_DIR)/relay

$(BUILD_DIR)/pi_protocol:
	mkdir -p $(BUILD_DIR)/pi_protocol

# Regenerate the committed pi-protocol files in src/pi_protocol/ from the
# vendored library in ext/pi-protocol/ (see src/pi_protocol/README.md). Only
# needed when the protocol changes; the normal build uses the checked-in files.
regen-pi:
	cd $(PROJECT_DIR)ext/pi-protocol && python3 python/generate.py config.yaml \
		--output-dir $(abspath $(SRC_DIR)/pi_protocol)
	cp $(PROJECT_DIR)ext/pi-protocol/src/pi-protocol.c $(SRC_DIR)/pi_protocol/pi-protocol.c

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS) $(COBJS) $(CDEPS) $(RELAY_OBJS) $(RELAY_DEPS) $(PI_OBJS) $(PI_DEPS)

-include $(DEPS)
-include $(CDEPS)
-include $(RELAY_DEPS)
-include $(PI_DEPS)
