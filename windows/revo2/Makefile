# =========================
# Environment (MSYS2 / MinGW)
# =========================
SHELL := /usr/bin/bash
export PATH := /mingw64/bin:/usr/bin:$(PATH)

# =========================
# Project paths
# =========================
SCRIPT_DIR := $(CURDIR)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)

RAW_LIB_DIR1   := $(DIST_DIR)/shared/win
RAW_LIB_DIR2   := $(DIST_DIR)/shared/win/kerneldlls
RAW_HEADER_DIR := $(DIST_DIR)/include

# =========================
# Path conversion (cygpath)
# =========================
ifeq ($(shell command -v cygpath 2>/dev/null),)
    LIB_DIR1   := $(RAW_LIB_DIR1)
    LIB_DIR2   := $(RAW_LIB_DIR2)
    HEADER_DIR := $(RAW_HEADER_DIR)
else
    LIB_DIR1   := $(shell cygpath -u "$(RAW_LIB_DIR1)")
    LIB_DIR2   := $(shell cygpath -u "$(RAW_LIB_DIR2)")
    HEADER_DIR := $(shell cygpath -u "$(RAW_HEADER_DIR)")
endif

INCLUDE := -I$(HEADER_DIR)

# =========================
# Build target selection
# =========================
# Default target
TARGET ?= revo2_canfd

# Target-specific config
ifeq ($(TARGET),revo2_canfd)
    CPP_SOURCE := revo2_canfd.cpp
    CAN_LIB    := -lzlgcan
else ifeq ($(TARGET),revo2_canfd_zqwl)
    CPP_SOURCE := revo2_canfd_zqwl.cpp
    CAN_LIB    := -lzqwlcan
else
    $(error Unsupported TARGET: $(TARGET))
endif

BIN := $(TARGET).exe

COMMON_LIBS := -L$(LIB_DIR1) -L$(LIB_DIR2) -lbc_stark_sdk $(CAN_LIB)

# =========================
# Compiler
# =========================
CXX := $(shell command -v g++ 2>/dev/null || echo g++)
CXXFLAGS := -g -Wall -std=c++17

# =========================
# Rules
# =========================
all: $(BIN)

$(BIN): $(CPP_SOURCE)
	@echo "Building target: $(TARGET)"
	@echo "Compiler: $(CXX)"
	@echo "Source: $(CPP_SOURCE)"
	@echo "Libs: $(CAN_LIB)"
	@bash -c 'export PATH="/mingw64/bin:/usr/bin:$$PATH"; \
		$(CXX) $(CXXFLAGS) $(INCLUDE) $< -o $@ $(COMMON_LIBS)'
	@echo "Built -> $@"

run: $(BIN)
	@echo "Running $(BIN)..."
	PATH="$(LIB_DIR1):$(LIB_DIR2):$$PATH" ./$(BIN)

clean:
	rm -f *.exe

.PHONY: all run clean
