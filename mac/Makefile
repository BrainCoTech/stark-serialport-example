# 设置变量
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../dist)
DEP_DIR := $(abspath $(SCRIPT_DIR)/../deps/libuv)

# 设置编译器和 pkg-config 工具
CC := gcc
CXX := g++
PKG_CONFIG := pkg-config

# 获取编译选项和链接选项
CFLAGS := -g -Wall $(shell $(PKG_CONFIG) --cflags libserialport)
CXXFLAGS := -g -Wall -std=c++11 $(shell $(PKG_CONFIG) --cflags libserialport)
LIBS := $(shell $(PKG_CONFIG) --libs libserialport) -rdynamic

SOURCES := $(wildcard *.c) $(wildcard *.cpp)
BINARIES := $(SOURCES:.c=) $(SOURCES:.cpp=)
# BINARIES := $(SOURCES:.cpp=)

# 编译规则，支持同时编译 .c 和 .cpp 文件
%: %.c
	$(CC) $(CFLAGS) $< \
		-I$(DIST_DIR)/include \
		-I$(DEP_DIR)/include \
		-L$(DIST_DIR)/mac/static \
		-lstark \
		-framework Foundation \
		$(LIBS) \
		-o $@.exe
	@echo "\033[1;33m[mac][static] build $@.c done\033[0m"

%: %.cpp
	$(CXX) $(CXXFLAGS) $< \
		-I$(DIST_DIR)/include \
		-I$(DEP_DIR)/include \
		-L$(DIST_DIR)/mac/static \
		-lstark \
		-framework Foundation \
		$(LIBS) \
		-o $@.exe
	@echo "\033[1;33m[mac][static] build $@.cpp done\033[0m"

# 默认目标
all: $(BINARIES)

# 清理目标
clean:
	rm -f $(addsuffix .exe,$(BINARIES))
	rm -fR $(addsuffix .exe.dSYM,$(BINARIES))

# 运行目标
run_serial:
	./rs485.exe

run_ota:
	./ota.exe

# 伪目标声明，确保不会与同名文件冲突
.PHONY: clean all run run_serial run_ota
