CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -MMD -MP $(shell pkg-config --cflags opencv4 realsense2)
LDFLAGS := $(shell pkg-config --libs opencv4 realsense2)

PROJECT_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR := $(PROJECT_DIR)src
INCLUDE_DIR := $(PROJECT_DIR)include
BUILD_DIR ?= $(PROJECT_DIR)build

CXXFLAGS += -I$(INCLUDE_DIR)

# USE_VO=1 builds in visual odometry (make USE_VO=1); default is without it,
# which excludes visual_odometry.cpp and the code that uses it.
USE_VO ?= 0

TARGET := $(BUILD_DIR)/depthcam
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
ifeq ($(USE_VO),1)
CXXFLAGS += -DUSE_VO
else
SRCS := $(filter-out $(SRC_DIR)/visual_odometry.cpp,$(SRCS))
endif
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)

-include $(DEPS)
