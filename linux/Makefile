# 设置变量
SCRIPT_DIR := $(shell pwd)
DIST_DIR := $(abspath $(SCRIPT_DIR)/../dist)

# 设置编译器和 pkg-config 工具
CC := gcc
PKG_CONFIG := pkg-config

# 获取编译选项和链接选项
CFLAGS := -Wall $(shell $(PKG_CONFIG) --cflags libserialport)
LIBS := $(shell $(PKG_CONFIG) --libs libserialport)

SOURCES = $(wildcard *.c)
BINARIES = $(SOURCES:.c=)

%: %.c
	$(CC) $(CFLAGS) $< \
	    -I$(DIST_DIR)/include \
	    -L$(DIST_DIR)/linux/static \
	    -lstark -lm -lpthread -lrt -lstdc++ -ldl \
	$(LIBS) \
	-o $@.exe
	@echo "\033[1;33m[mac][static] build done\033[0m"

all: $(BINARIES)

clean:
	rm -f $(addsuffix .exe,$(BINARIES))
	rm -fR $(addsuffix .exe.dSYM,$(BINARIES))

run:
	./rs485.exe

# 声明伪目标，确保它不会与同名文件冲突
.PHONY: clean all run