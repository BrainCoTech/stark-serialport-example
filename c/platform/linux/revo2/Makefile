# Linux-only Revo2 Examples (EtherCAT)
# For cross-platform examples, use ../../revo2/
.DEFAULT_GOAL := all

# Directory setup
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)
COMMON_DIR := $(abspath $(SCRIPT_DIR)/../../../common)

# Compiler settings
CXX := g++
CXXFLAGS := -g -Wall -std=c++17

# Include paths
INCLUDE := -I/usr/local/include -I$(HEADER_DIR) -I$(COMMON_DIR)

# Libraries
BASE_LIB_DIRS := $(LIB_DIR) /usr/local/lib /usr/lib
RPATH_DIRS := $(LIB_DIR):/usr/local/lib:/usr/lib
BASE_LIBS := $(foreach d,$(BASE_LIB_DIRS),-L$(d)) -Wl,-rpath,$(RPATH_DIRS) -lbc_stark_sdk

# Common object files
COMMON_OBJ := $(COMMON_DIR)/stark_common.o

# Build common library
$(COMMON_OBJ): $(COMMON_DIR)/stark_common.cpp $(COMMON_DIR)/stark_common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[linux] built stark_common.o\033[0m"

# EtherCAT example
revo2_ethercat.exe: revo2_ethercat.cpp $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(COMMON_OBJ) $(INCLUDE) $(BASE_LIBS) -lethercat
	@echo "\033[1;32m[linux] Built $@\033[0m"

all: revo2_ethercat.exe
	@echo "\033[1;32m[linux] EtherCAT build done\033[0m"

run: revo2_ethercat.exe
	@echo "\033[1;36m[Running] revo2_ethercat.exe\033[0m"
	@LD_LIBRARY_PATH=$(LIB_DIR):$$LD_LIBRARY_PATH ./revo2_ethercat.exe

clean:
	rm -f *.exe $(COMMON_OBJ)

help:
	@echo "\033[1;32m=== Linux Revo2 EtherCAT Makefile ===\033[0m"
	@echo "Build: make"
	@echo "Run:   make run"
	@echo "Clean: make clean"
	@echo ""
	@echo "Note: Requires libethercat.so installed"
	@echo "For cross-platform examples, use ../../revo2/"

.PHONY: all run clean help
