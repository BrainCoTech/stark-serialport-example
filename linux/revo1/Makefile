# Variable definitions
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)

# Header search paths
INCLUDE := -I/usr/local/include -I$(HEADER_DIR)

# Compilers
CC := gcc
CXX := g++

# Compile options
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17  # Use C++17 standard

# ---------------- Mode selection ----------------
# MODE/mod options: none / can / ethercat
# Default: none
# Both MODE and mod are supported, with mod taking precedence if provided
ifneq ($(mod),)
  MODE := $(mod)
else
  MODE ?= none
endif
# Convert to lowercase to make it case-insensitive
MODE := $(shell echo $(MODE) | tr '[:upper:]' '[:lower:]')

ifeq ($(MODE),none)
  CPP_SOURCES := $(wildcard *.cpp)
  CPP_SOURCES := $(filter-out revo1_can%, $(CPP_SOURCES))
  COMMON_LIBS := -L/usr/local/lib -L$(LIB_DIR) \
                 -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) \
                 -lbc_stark_sdk -lm -lpthread -lrt -ldl
endif

ifeq ($(MODE),can)
  CPP_SOURCES := $(wildcard revo1_can_*.cpp)
  COMMON_LIBS := -L/usr/local/lib -L$(LIB_DIR) \
                 -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) \
                 -lbc_stark_sdk -lm -lpthread -lrt -ldl \
                 -lusbcanfd  # ZLG USB-CAN so库
endif

# ---------------- Sources and targets ----------------
C_SOURCES := $(wildcard *.c)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# Build C files
%: %.c
	$(CC) $(CFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.c done\033[0m"

# Build C++ files
%: %.cpp
	$(CXX) $(CXXFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"

# Default target
all: $(BINARIES)
	@echo "\033[1;32m[linux] build done, MODE=$(MODE)\033[0m"

# Clean target
clean:
	rm -f *.exe

# Runtime support
export LD_LIBRARY_PATH := $(LIB_DIR):$(LD_LIBRARY_PATH)

run_%:
	./$*.exe

# Phony targets
.PHONY: clean all run
