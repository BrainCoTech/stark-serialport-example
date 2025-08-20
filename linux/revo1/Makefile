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

# COMMON_LIBS := -L/usr/local/lib -L$(LIB_DIR) -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) -lbc_stark_sdk -lm -lpthread -lrt -ldl # -rdynamic
COMMON_LIBS := -L/usr/local/lib -L$(LIB_DIR) -Wl,-rpath,/usr/local/lib:/usr/lib:$(LIB_DIR) -lbc_stark_sdk -lm -lpthread -lrt -ldl -lusbcanfd # 添加 ZLG USB-CAN FD 库
# sudo ln -s libusbcanfd.so libusbcanfd.so.1.0.10

# 获取所有 .c 和 .cpp 文件
C_SOURCES := $(wildcard *.c)
# CPP_SOURCES := $(wildcard *.cpp)
# CPP_SOURCES := $(filter-out revo1_can.cpp, $(CPP_SOURCES))
CPP_SOURCES := revo1_can.cpp
BINARIES := $(C_SOURCES:.c=) $(CPP_SOURCES:.cpp=)

# 编译 C++ 文件
%: %.cpp
	$(CXX) $(CXXFLAGS) -o $@.exe $< \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"

# 默认目标
all: $(BINARIES)

# 清理目标
clean:
	rm -f $(addsuffix .exe,$(BINARIES))

# export LD_LIBRARY_PATH := $(LIB_DIR):$(LD_LIBRARY_PATH)
run_%:
	export LD_LIBRARY_PATH=$(LIB_DIR):$(LD_LIBRARY_PATH) &&  ./$*.exe

# 伪目标声明，确保不会与同名文件冲突
.PHONY: clean all run
