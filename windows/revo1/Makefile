# Use bash shell for MSYS2/MinGW environments
# In MINGW64 terminal, bash is typically at /usr/bin/bash
SHELL := /usr/bin/bash

# Export PATH to ensure MinGW tools are found
# Include MSYS2/MinGW locations
export PATH := /mingw64/bin:/usr/bin:$(PATH)

# Variable definitions
SCRIPT_DIR := $(CURDIR)
$(info Current dir: $(SCRIPT_DIR))
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)

# Convert to paths recognizable by MSYS2/MinGW
RAW_LIB_DIR1 := $(DIST_DIR)/shared/win
RAW_LIB_DIR2 := $(DIST_DIR)/shared/win/kerneldlls
RAW_HEADER_DIR := $(DIST_DIR)/include

# Try to use cygpath if available (MSYS2/MinGW), otherwise use paths as-is
ifeq ($(shell command -v cygpath 2>/dev/null || echo ""),)
    # cygpath not available, use paths directly (for native Windows)
    LIB_DIR1 := $(RAW_LIB_DIR1)
    LIB_DIR2 := $(RAW_LIB_DIR2)
    HEADER_DIR := $(RAW_HEADER_DIR)
else
    # cygpath available, convert paths
    LIB_DIR1 := $(shell cygpath -u "$(RAW_LIB_DIR1)")
    LIB_DIR2 := $(shell cygpath -u "$(RAW_LIB_DIR2)")
    HEADER_DIR := $(shell cygpath -u "$(RAW_HEADER_DIR)")
endif

INCLUDE := -I$(HEADER_DIR)
COMMON_LIBS := -L$(LIB_DIR1) -L$(LIB_DIR2) -lbc_stark_sdk -lzlgcan # ZLG CAN/CANFD dll

# Check if compiler is available and get full path
CC := gcc
CXX := g++
# Find g++ full path - try multiple methods and paths
GXX_PATH := $(shell bash -c 'export PATH="/mingw64/bin:/usr/bin:$$PATH" && command -v g++ 2>/dev/null || which g++ 2>/dev/null || echo ""')
ifneq ($(GXX_PATH),)
    CXX := $(GXX_PATH)
    $(info Using compiler: $(CXX))
else
    # Try common MinGW/MSYS2 paths
    ifneq ($(wildcard /mingw64/bin/g++.exe),)
        CXX := /mingw64/bin/g++.exe
        $(info Using compiler: $(CXX))
    else ifneq ($(wildcard /usr/bin/g++.exe),)
        CXX := /usr/bin/g++.exe
        $(info Using compiler: $(CXX))
    else ifneq ($(wildcard /c/msys64/mingw64/bin/g++.exe),)
        CXX := /c/msys64/mingw64/bin/g++.exe
        $(info Using compiler: $(CXX))
    else
        # Last resort: use g++ and let the shell find it with proper PATH
        $(warning Warning: g++ not found in standard paths. Will try to find it at compile time.)
        CXX := g++
    endif
endif

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
	@echo "Compiling with: $(CXX)"
	@echo "Library dirs: $(LIB_DIR1) $(LIB_DIR2)"
	@echo "Include dir: $(HEADER_DIR)"
	@bash -c 'export PATH="/mingw64/bin:/usr/bin:$$PATH"; \
		GXX="$(CXX)"; \
		if [ "$$GXX" = "g++" ]; then \
			GXX=$$(command -v g++ 2>/dev/null || which g++ 2>/dev/null || echo "g++"); \
		fi; \
		if [ -z "$$GXX" ] || [ "$$GXX" = "g++" ]; then \
			if [ -f "/mingw64/bin/g++.exe" ]; then \
				GXX="/mingw64/bin/g++.exe"; \
			fi; \
		fi; \
		$$GXX $(CXXFLAGS) $(INCLUDE) $< -o $@ $(COMMON_LIBS)'
	@echo "Built $< -> $@"

run_%:
	@echo "Running $@..."
	@echo "Using library paths: $(LIB_DIR1) and $(LIB_DIR2)"
	PATH="$(LIB_DIR1):$(LIB_DIR2):$$PATH" ./$(subst run_,,$@).exe
	@echo "Done running $@"

run: run_revo1_can

.PHONY: clean all run
