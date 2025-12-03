# Set variables
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)
# Header file directory
INCLUDE := -I/usr/local/include -I$(HEADER_DIR)
# Define common library path
COMMON_LIBS := -L/usr/local/lib -L$(LIB_DIR) -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) -lethercat -lbc_stark_sdk -lm -lpthread -lrt -ldl # -rdynamic

# Set compiler
CC := gcc
CXX := g++

# Compilation options
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17  # Specify C++17 standard

# Get all .c and .cpp files
C_SOURCES := $(wildcard *.c)
CPP_SOURCES := $(wildcard *.cpp)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# Compile C++ files
%: %.cpp
	$(CXX) $(CXXFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"
# 	sudo setcap cap_sys_nice,cap_net_raw=eip $@.exe

# Default target
all: $(BINARIES)

# Clean target
clean:
	rm -f $(addsuffix .exe,$(BINARIES))

# Run target
run_%:
	./$*.exe

# Phony target declaration to ensure no conflict with files of the same name
.PHONY: clean all run
