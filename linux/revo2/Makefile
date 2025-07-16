# 设置变量
SCRIPT_DIR := $(shell pwd)
# DIST_DIR := $(abspath $(SCRIPT_DIR)/../../lib)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)

# 设置编译器
CC := gcc
CXX := g++

# 编译选项
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17  # 指定 C++17 标准

# 获取所有 .c 和 .cpp 文件
C_SOURCES := $(wildcard *.c)
CPP_SOURCES := $(wildcard *.cpp)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# 编译 C 文件
%: %.c
	$(CC) $(CFLAGS) -o $@.exe $< \
		-I$(HEADER_DIR) \
		-L$(LIB_DIR) \
		-Wl,-rpath,$(LIB_DIR) \
		-lbc_stark_sdk \
		-lm -lpthread -lrt -ldl \
		-rdynamic
	@echo "\033[1;33m[linux] build $@.c done\033[0m"

# 编译 C++ 文件
%: %.cpp
	$(CXX) $(CXXFLAGS) -o $@.exe $< \
		-I$(HEADER_DIR) \
		-L$(LIB_DIR) \
		-Wl,-rpath,$(LIB_DIR) \
		-lbc_stark_sdk \
		-lm -lpthread -lrt -ldl \
		-rdynamic
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"

# 默认目标
all: $(BINARIES)

# 清理目标
clean:
	rm -f $(addsuffix .exe,$(BINARIES))

# 运行目标
run:
	export LD_LIBRARY_PATH=$(LIB_DIR):$(LD_LIBRARY_PATH) && ./revo2_ctrl.exe

run_multi:
	export LD_LIBRARY_PATH=$(LIB_DIR):$(LD_LIBRARY_PATH) && ./revo2_ctrl_multi.exe

run_customed_modbus:
	export LD_LIBRARY_PATH=$(LIB_DIR):$(LD_LIBRARY_PATH) && ./revo2_customed_modbus.exe

run_customed_modbus_async:
	export LD_LIBRARY_PATH=$(LIB_DIR):$(LD_LIBRARY_PATH) && ./revo2_customed_modbus_async.exe

run_ethercat:
	export DYLD_LIBRARY_PATH=$(LIB_DIR):$(DYLD_LIBRARY_PATH) && sudo setcap cap_sys_nice,cap_net_raw=eip revo2_ethercat.exe && ./revo2_ethercat.exe

# 伪目标声明，确保不会与同名文件冲突
.PHONY: clean all run
