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
CXXFLAGS := -g -Wall -std=c++17

# Mode selection: modbus(default) / can / all
MODE := $(shell echo $(if $(mod),$(mod),$(if $(MODE),$(MODE),modbus)) | tr '[:upper:]' '[:lower:]')

# CAN backend: compile all, select at runtime
# Use STARK_NO_CAN=1 to disable CAN support
ifeq ($(STARK_NO_CAN),1)
  CAN_DEFS := -DSTARK_NO_CAN
  CAN_COMMON_OBJ :=
else
  CAN_DEFS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=1
  CAN_COMMON_OBJ := $(COMMON_DIR)/can_common.o
endif

# Base libraries (ZLG loaded dynamically, no -lusbcanfd needed)
BASE_LIB_DIRS := $(LIB_DIR) /usr/local/lib /usr/lib
RPATH_DIRS := $(LIB_DIR):/usr/local/lib:/usr/lib
BASE_LIBS := $(foreach d,$(BASE_LIB_DIRS),-L$(d)) -Wl,-rpath,$(RPATH_DIRS) -lbc_stark_sdk -ldl

ifeq ($(MODE),modbus)
  CPP_SOURCES := $(filter-out revo1_can%,$(wildcard *.cpp))
  COMMON_LIBS := $(BASE_LIBS)
else ifeq ($(MODE),can)
  CPP_SOURCES := $(wildcard revo1_can*.cpp)
  COMMON_LIBS := $(BASE_LIBS)
else ifeq ($(MODE),all)
  CPP_SOURCES := $(wildcard revo1*.cpp)
  COMMON_LIBS := $(BASE_LIBS)
endif

# ---------------- Sources and targets ----------------
C_SOURCES := $(wildcard *.c)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# Common library object files
COMMON_OBJ := $(COMMON_DIR)/stark_common.o
DFU_COMMON_OBJ := $(COMMON_DIR)/dfu_common.o

# Build common library
$(COMMON_OBJ): $(COMMON_DIR)/stark_common.cpp $(COMMON_DIR)/stark_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built stark_common.o\033[0m"

# Build CAN common library
ifneq ($(CAN_COMMON_OBJ),)
$(CAN_COMMON_OBJ): $(COMMON_DIR)/can_common.cpp $(COMMON_DIR)/can_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built can_common.o\033[0m"
endif

# Build DFU common library
$(DFU_COMMON_OBJ): $(COMMON_DIR)/dfu_common.cpp $(COMMON_DIR)/dfu_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built dfu_common.o\033[0m"

# Build rules
revo1_dfu.exe: revo1_dfu.cpp $(COMMON_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo1_can_dfu.exe: revo1_can_dfu.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo1_can.exe: revo1_can.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo1_can%.exe: revo1_can%.cpp $(COMMON_OBJ) $(CAN_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo1_%.exe: revo1_%.cpp $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

revo1_%.exe: revo1_%.c $(COMMON_OBJ)
	$(CC) $(CFLAGS) -o $@ $< $(COMMON_OBJ) $(INCLUDE) $(COMMON_LIBS)
	@echo "\033[1;32m[Built] $@\033[0m"

# Default target
all: $(addsuffix .exe,$(BINARIES))
	@echo "\033[1;32m[linux] build done, MODE=$(MODE)\033[0m"

# Clean target
clean:
	rm -f *.exe $(COMMON_OBJ) $(CAN_COMMON_OBJ) $(DFU_COMMON_OBJ)

# Runtime support
export LD_LIBRARY_PATH := $(LIB_DIR):/usr/local/lib:$(LD_LIBRARY_PATH)
LDFLAGS += -Wl,-rpath,/usr/local/lib:$(LIB_DIR)

# Run targets
run_revo1_can%:
	@$(MAKE) MODE=can $(patsubst run_%,%,$@).exe
	@echo "\033[1;36m[Running] $(patsubst run_%,%,$@).exe\033[0m"
	@./$(patsubst run_%,%,$@).exe

run_%: %.exe
	@echo "\033[1;36m[Running] $<\033[0m"
	@./$<

.PHONY: run
run:
	@target="$(filter-out run,$(MAKECMDGOALS))"; \
	if [ -z "$$target" ]; then \
		echo "\033[1;31m[ERROR] Usage: make run <target>\033[0m"; \
		exit 1; \
	else \
		case "$$target" in \
			revo1_can*) $(MAKE) MODE=can run_$$target ;; \
			*) $(MAKE) run_$$target ;; \
		esac; \
	fi

ifneq ($(filter run,$(MAKECMDGOALS)),)
$(filter-out run run_%,$(MAKECMDGOALS)):
	@:
endif

help:
	@echo "\033[1;32m=== Revo1 Makefile ===\033[0m"
	@echo "Build: make [MODE=modbus|can|all]"
	@echo "       make STARK_NO_CAN=1    # Disable CAN support"
	@echo "Run:   make run <target>"
	@echo "Clean: make clean"
	@echo ""
	@echo "CAN backend (runtime selection):"
	@echo "  Default: SocketCAN"
	@echo "  STARK_CAN_BACKEND=zlg ./revo1_can.exe  # Use ZLG"
	@echo "  STARK_ZLG_LIB_PATH=/path/to/lib ./...  # Custom ZLG lib path"

.PHONY: clean all help run
