# Default target
.DEFAULT_GOAL := all

# Variable definitions
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)
COMMON_DIR := $(abspath $(SCRIPT_DIR)/../common)

# Header search paths
INCLUDE := -I/usr/local/include -I$(HEADER_DIR) -I$(COMMON_DIR)

# Compilers
CC := gcc
CXX := g++

# Compile options
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17  # Use C++17 standard

# Mode selection: modbus(default) / can / ethercat / all
MODE := $(shell echo $(if $(mod),$(mod),$(if $(MODE),$(MODE),modbus)) | tr '[:upper:]' '[:lower:]')

# CAN backend selection: zlg / socketcan / both(default)
CAN_BACKEND := $(shell echo $(if $(CAN_BACKEND),$(CAN_BACKEND),$(if $(STARK_CAN_BACKEND),$(STARK_CAN_BACKEND),both)) | tr '[:upper:]' '[:lower:]')
CAN_COMMON_OBJ := $(COMMON_DIR)/can_common.$(CAN_BACKEND).o

# Base libraries
ZLG_LIB_DIR ?=
BASE_LIB_DIRS := /usr/local/lib /usr/lib $(LIB_DIR) $(ZLG_LIB_DIR)
BASE_LIBS := $(foreach d,$(BASE_LIB_DIRS),-L$(d)) -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) -lbc_stark_sdk

ifeq ($(CAN_BACKEND),socketcan)
  CAN_LIBS :=
  CAN_DEFS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=1
else ifeq ($(CAN_BACKEND),both)
  CAN_LIBS := -lusbcanfd
  CAN_DEFS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=1
else
  CAN_LIBS := -lusbcanfd
  CAN_DEFS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=0
endif

# Validate ZLG library when needed (skip for clean-only targets)
ifneq ($(filter clean,$(MAKECMDGOALS)),clean)
  ifneq ($(filter zlg both,$(CAN_BACKEND)),)
    ZLG_LIB_FILE := $(firstword $(foreach d,$(BASE_LIB_DIRS),$(wildcard $(d)/libusbcanfd.so)))
    ifeq ($(ZLG_LIB_FILE),)
      $(error libusbcanfd.so not found. Run ./download-lib.sh or set ZLG_LIB_DIR=<path> to ZLG library.)
    endif
  endif
endif

# Mode-specific settings
ifeq ($(MODE),modbus)
  CPP_SOURCES := $(filter-out revo2_can% revo2_ethercat%,$(wildcard revo2*.cpp))
  COMMON_LIBS := $(BASE_LIBS)
else ifeq ($(MODE),can)
  CPP_SOURCES := $(wildcard revo2_can*.cpp)
  COMMON_LIBS := $(BASE_LIBS) $(CAN_LIBS)
else ifeq ($(MODE),ethercat)
  CPP_SOURCES := $(wildcard revo2_ethercat*.cpp)
  COMMON_LIBS := $(BASE_LIBS) -lethercat
else ifeq ($(MODE),all)
  CPP_SOURCES := $(wildcard revo2*.cpp)
  COMMON_LIBS := $(BASE_LIBS) $(CAN_LIBS) -lethercat
endif

# ---------------- Sources and targets ----------------
C_SOURCES := $(wildcard revo2*.c)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# Common library object files
COMMON_OBJ := $(COMMON_DIR)/stark_common.o
DFU_COMMON_OBJ := $(COMMON_DIR)/dfu_common.o

# Build common library (C++ file)
$(COMMON_OBJ): $(COMMON_DIR)/stark_common.cpp $(COMMON_DIR)/stark_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built common library stark_common.o\033[0m"

# Build CAN common library (C++ file)
$(CAN_COMMON_OBJ): $(COMMON_DIR)/can_common.cpp $(COMMON_DIR)/can_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built CAN common library can_common.o\033[0m"

# Build DFU common library (C++ file)
$(DFU_COMMON_OBJ): $(COMMON_DIR)/dfu_common.cpp $(COMMON_DIR)/dfu_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built DFU common library dfu_common.o\033[0m"

# Build rules for .exe files (specific rules first, then patterns)

# Specific rules for special cases
revo2_dfu.exe: revo2_dfu.cpp $(COMMON_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

# Pattern rules for CAN/CANFD files
revo2_can_dfu.exe: revo2_can_dfu.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo2_canfd_dfu.exe: revo2_canfd_dfu.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo2_can%.exe: revo2_can%.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo2_canfd%.exe: revo2_canfd%.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

# General pattern rules
revo2_%.exe: revo2_%.cpp $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo2_%.exe: revo2_%.c $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $< $(COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

# Default target
all: $(addsuffix .exe,$(BINARIES))
	@echo "\033[1;32m[linux] build done, MODE=$(MODE)\033[0m"

# Clean target
clean:
	rm -f *.exe $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ) $(COMMON_DIR)/can_common.o

# Runtime support
export LD_LIBRARY_PATH := $(LIB_DIR):$(LD_LIBRARY_PATH)

# Smart run targets with auto-build
run_revo2_can% run_revo2_canfd%:
	@$(MAKE) MODE=can $(patsubst run_%,%,$@).exe
	@echo "\033[1;36m[Running] $(patsubst run_%,%,$@).exe\033[0m"
	@./$(patsubst run_%,%,$@).exe

run_revo2_ethercat%:
	@$(MAKE) MODE=ethercat $(patsubst run_%,%,$@).exe
	@echo "\033[1;36m[Running] $(patsubst run_%,%,$@).exe\033[0m"
	@./$(patsubst run_%,%,$@).exe

run_%: %.exe
	@echo "\033[1;36m[Running] $<\033[0m"
	@./$<

# Generic run target: make run <target>
.PHONY: run
run:
	@target="$(filter-out run,$(MAKECMDGOALS))"; \
	if [ -z "$$target" ]; then \
		echo "\033[1;31m[ERROR] Usage: make run <target>\033[0m"; \
		exit 1; \
	else \
		case "$$target" in \
			revo2_can*|revo2_canfd*) $(MAKE) MODE=can run_$$target ;; \
			revo2_ethercat*) $(MAKE) MODE=ethercat run_$$target ;; \
			*) $(MAKE) run_$$target ;; \
		esac; \
	fi

# Prevent make from building targets when used with 'run'
ifneq ($(filter run,$(MAKECMDGOALS)),)
$(filter-out run run_%,$(MAKECMDGOALS)):
	@:
endif

help:
	@echo "\033[1;32m=== Revo2 Makefile ===\033[0m"
	@echo "Modes: make [MODE=modbus|can|ethercat|all]"
	@echo "CAN backend: make [CAN_BACKEND=zlg|socketcan|both]"
	@echo "ZLG library dir: make [ZLG_LIB_DIR=/path/to/lib]"
	@echo "Run:   make run_<target> OR make run <target>"
	@echo "Clean: make clean"

.PHONY: all clean help run
