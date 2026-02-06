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
# CAN_BACKEND: zlg, socketcan, or unset (default: ZQWL built-in)
#
# Supported backends:
# 1. ZQWL (default): SDK built-in support, no external dependencies
#    - Usage: make (no CAN_BACKEND needed)
#    - Platforms: Linux, macOS, Windows
#
# 2. ZLG: ZLG USB-CANFD adapter (device type 41)
#    - Usage: make CAN_BACKEND=zlg
#    - Platforms: Linux, Windows (not supported on macOS)
#    - Requires: libusbcanfd.so/.dll from https://manual.zlg.cn/web/#/146
#      - Windows: libusbcanfd.dll (x64/x86 versions available)
#      - Linux: libusbcanfd.so (x86_64/aarch64 versions available)
#        - Depends on libusb-1.0:
#          Ubuntu/Debian: sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev
#          CentOS/RHEL:   sudo yum install libusb1-devel
#    - Config: Arbitration 1 Mbps, Data 5 Mbps
#
# 3. SocketCAN: Linux kernel CAN interface
#    - Usage: make CAN_BACKEND=socketcan
#    - Platforms: Linux only
#    - Requires: CAN interface configured (e.g., can0)
#
# Examples:
#   make                      # Use ZQWL (default)
#   make CAN_BACKEND=zlg      # Use ZLG USB-CANFD
#   make CAN_BACKEND=socketcan # Use SocketCAN (Linux)
#

# ZLG library directory (can be overridden)
ZLG_LIB_DIR ?=

ifeq ($(CAN_BACKEND),zlg)
    ifneq ($(OS),mac)
        CAN_CFLAGS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=0
        CAN_LIBS := -lusbcanfd
        # Add ZLG lib dir to search paths if specified
        ifneq ($(ZLG_LIB_DIR),)
            BASE_LIBS := -L$(LIB_DIR) -L$(ZLG_LIB_DIR) $(RPATH_FLAG) -Wl,-rpath,$(ZLG_LIB_DIR) -lbc_stark_sdk $(PLATFORM_LIBS) $(CAN_LIBS)
        else
            BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS) $(CAN_LIBS)
        endif
    else
        $(warning ZLG not supported on macOS, ignoring CAN_BACKEND=zlg)
        CAN_CFLAGS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=0
        CAN_LIBS :=
        BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS)
    endif
else ifeq ($(CAN_BACKEND),socketcan)
    ifeq ($(OS),linux)
        CAN_CFLAGS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=1
        CAN_LIBS :=
        BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS)
    else
        $(warning SocketCAN only available on Linux, ignoring CAN_BACKEND=socketcan)
        CAN_CFLAGS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=0
        CAN_LIBS :=
        BASE_LIBS := -L$(LIB_DIR) $(RPATH_FLAG) -lbc_stark_sdk $(PLATFORM_LIBS)
    endif
else
    # Default: no external CAN backend (use SDK built-in ZQWL)
    CAN_CFLAGS := -DSTARK_USE_ZLG=0 -DSTARK_USE_SOCKETCAN=0
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
