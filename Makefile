CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -MMD -MP $(shell pkg-config --cflags opencv4 realsense2)
LDFLAGS := $(shell pkg-config --libs opencv4 realsense2)

PROJECT_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR := $(PROJECT_DIR)src
INCLUDE_DIR := $(PROJECT_DIR)include
BUILD_DIR ?= $(PROJECT_DIR)build

CXXFLAGS += -I$(INCLUDE_DIR)

TARGET := $(BUILD_DIR)/depthcam
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
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
