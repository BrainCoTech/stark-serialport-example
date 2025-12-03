# SHELL := /usr/bin/bash

# Variable definitions
SCRIPT_DIR := $(CURDIR)
$(info Current dir: $(SCRIPT_DIR))
DIST_DIR := $(abspath $(SCRIPT_DIR)/../dist)

# Convert to paths recognizable by MSYS2/MinGW
RAW_LIB_DIR := $(DIST_DIR)/shared/win
RAW_HEADER_DIR := $(DIST_DIR)/include
LIB_DIR := $(shell cygpath -u "$(RAW_LIB_DIR)")
HEADER_DIR := $(shell cygpath -u "$(RAW_HEADER_DIR)")

INCLUDE := -I$(HEADER_DIR)
COMMON_LIBS := -L$(LIB_DIR) -lbc_stark_sdk -lzlgcan # ZQWL CAN/CANFD dll

CC := gcc
CXX := g++
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17
# CXXFLAGS := -v -g -Wall -std=c++17

CPP_SOURCES := revo1_can.cpp
# CPP_SOURCES := $(wildcard *.cpp)
CPP_OBJS := $(CPP_SOURCES:.cpp=.exe)
BINARIES := $(CPP_OBJS)

all: $(BINARIES)

clean:
	rm -f $(BINARIES)

%.exe: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) $< -o $@ $(COMMON_LIBS)
	@echo "Built $< -> $@"

run_%:
	@echo "Running $@..."
	@echo "Using library path: $(LIB_DIR)"
	PATH="$(LIB_DIR):$$PATH" ./$(subst run_,,$@).exe
	@echo "Done running $@"

run: run_revo1_can

.PHONY: clean all run
