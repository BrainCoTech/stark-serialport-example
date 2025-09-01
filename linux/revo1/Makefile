# 设置变量
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../../dist)
LIB_DIR := $(abspath $(DIST_DIR)/shared/linux)
HEADER_DIR := $(abspath $(DIST_DIR)/include)

# 头文件目录
INCLUDE := -I/usr/local/include -I$(HEADER_DIR)

# 设置编译器
CC := gcc
CXX := g++

# 编译选项
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17  # 指定 C++17 标准

# ---------------- 模式选择 ----------------
# MODE 可选：none / can
# 默认值：none
MODE ?= none

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

# ---------------- 文件与目标 ----------------
C_SOURCES := $(wildcard *.c)
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# 编译 C 文件
%: %.c
	$(CC) $(CFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.c done\033[0m"

# 编译 C++ 文件
%: %.cpp
	$(CXX) $(CXXFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"

# 默认目标
all: $(BINARIES)
	@echo "\033[1;32m[linux] build done, MODE=$(MODE)\033[0m"

# 清理目标
clean:
	rm -f *.exe

# 运行支持
export LD_LIBRARY_PATH := $(LIB_DIR):$(LD_LIBRARY_PATH)

run_%:
	./$*.exe

# 伪目标声明
.PHONY: clean all run
