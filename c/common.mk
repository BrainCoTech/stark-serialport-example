# Common Makefile for cross-platform builds (Linux/macOS/Windows)
# Include this file in subdirectory Makefiles
#
# Cross-compile for Windows: make TARGET=win
#
# Platform-specific libraries:
#   - Windows: ws2_32 (Winsock2 network library)
#   - macOS: Foundation framework
#   - Linux: pthread
#
# Compiler flags:
#   -g      : Generate debug symbols for GDB/LLDB debugging
#   -Wall   : Enable all common compiler warnings
#   -std=c++17 : Use C++17 standard (current mainstream, widely supported)

# Cross-compile target override (TARGET=win for Windows cross-compile)
ifdef TARGET
ifeq ($(TARGET),win)
    OS := win
    LIB_SUBDIR := shared/win
    PLATFORM_LIBS := -lws2_32
    RPATH_FLAG :=
    RUN_ENV :=
    EXE_EXT := .exe
    # MinGW cross-compiler
    CC := x86_64-w64-mingw32-gcc
    CXX := x86_64-w64-mingw32-g++
    CROSS_COMPILE := 1
endif
endif

# Detect native OS (if not cross-compiling)
ifndef CROSS_COMPILE
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    OS := mac
    LIB_SUBDIR := shared/mac
    PLATFORM_LIBS := -framework Foundation
    RPATH_FLAG = -Wl,-rpath,$(LIB_DIR)
    RUN_ENV = DYLD_LIBRARY_PATH=$(LIB_DIR):$$DYLD_LIBRARY_PATH
else ifeq ($(OS),Windows_NT)
    OS := win
    LIB_SUBDIR := shared/win
    PLATFORM_LIBS := -lws2_32
    RPATH_FLAG :=
    RUN_ENV :=
    EXE_EXT := .exe
else
    OS := linux
    LIB_SUBDIR := shared/linux
    PLATFORM_LIBS := -lpthread
    RPATH_FLAG = -Wl,-rpath,$(LIB_DIR):/usr/local/lib:/usr/lib
    RUN_ENV = LD_LIBRARY_PATH=$(LIB_DIR):/usr/local/lib:$$LD_LIBRARY_PATH
endif
# Compilers for native build
CC := gcc
CXX := g++
endif

# Directory setup
SCRIPT_DIR := $(shell pwd)
DIST_DIR ?= $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/$(LIB_SUBDIR))
HEADER_DIR := $(abspath $(DIST_DIR)/include)
COMMON_DIR := $(abspath $(SCRIPT_DIR)/../common)

# Header search paths
INCLUDE := -I$(HEADER_DIR) -I$(COMMON_DIR)
ifeq ($(OS),linux)
    INCLUDE += -I/usr/local/include
endif

# Compile options
ifeq ($(OS),mac)
    CFLAGS := -Wall
    CXXFLAGS := -Wall -std=c++17
else
    CFLAGS := -g -Wall
    CXXFLAGS := -g -Wall -std=c++17
endif

# CAN backend configuration
# ============================================================================
#
# All backends compiled by default (runtime selection via CLI or environment)
#
# Runtime selection:
#   - CLI: -s/-S (SocketCAN), -z/-Z (ZLG), -c/-f (ZQWL)
#   - Environment: STARK_CAN_BACKEND=socketcan|zlg
#
# Compile options:
#   - make                    # All backends (SocketCAN + ZLG on Linux)
#   - make STARK_NO_CAN=1     # Disable CAN support entirely
#
# Note: ZLG uses dynamic loading (dlopen), no compile-time dependency
#
# ============================================================================

# Check for STARK_NO_CAN first
ifeq ($(STARK_NO_CAN),1)
    CAN_CFLAGS := -DSTARK_NO_CAN
    CAN_LIBS :=
    BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS)
else
    # Compile with all available backends
    ifeq ($(OS),linux)
        # Linux: SocketCAN + ZLG
        CAN_CFLAGS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=1
    else ifeq ($(OS),win)
        # Windows: ZLG only
        CAN_CFLAGS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=0
    else
        # macOS: Neither (ZQWL only via SDK)
        CAN_CFLAGS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=0
    endif
    CAN_LIBS :=
    BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS)
endif

CFLAGS += $(CAN_CFLAGS)
CXXFLAGS += $(CAN_CFLAGS)

# Common object files (use different suffix for cross-compile to avoid conflicts)
ifdef CROSS_COMPILE
    OBJ_SUFFIX := _win.o
else
    OBJ_SUFFIX := .o
endif
COMMON_OBJ := $(COMMON_DIR)/stark_common$(OBJ_SUFFIX)
DFU_COMMON_OBJ := $(COMMON_DIR)/dfu_common$(OBJ_SUFFIX)
CAN_COMMON_OBJ := $(COMMON_DIR)/can_common$(OBJ_SUFFIX)

# Build common library
ifneq ($(wildcard $(COMMON_DIR)/stark_common.cpp),)
$(COMMON_OBJ): $(COMMON_DIR)/stark_common.cpp $(COMMON_DIR)/stark_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[$(OS)] built stark_common$(OBJ_SUFFIX)\033[0m"
endif

ifneq ($(wildcard $(COMMON_DIR)/dfu_common.cpp),)
$(DFU_COMMON_OBJ): $(COMMON_DIR)/dfu_common.cpp $(COMMON_DIR)/dfu_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[$(OS)] built dfu_common$(OBJ_SUFFIX)\033[0m"
endif

ifneq ($(wildcard $(COMMON_DIR)/can_common.cpp),)
$(CAN_COMMON_OBJ): $(COMMON_DIR)/can_common.cpp $(COMMON_DIR)/can_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[$(OS)] built can_common$(OBJ_SUFFIX)\033[0m"
endif

# Clean common objects
clean-common:
	rm -f $(COMMON_DIR)/*.o 2>/dev/null || true

.PHONY: clean-common
