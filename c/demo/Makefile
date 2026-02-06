# Demo Examples - Cross-platform Makefile
# Auto-detect protocol and device type
.DEFAULT_GOAL := all

include ../common.mk

LIBS := $(BASE_LIBS)

ifeq ($(OS),linux)
    EXTRA_LDFLAGS := -rdynamic
else
    EXTRA_LDFLAGS :=
endif

# All common objects for hand_demo (includes can_common for SocketCAN support)
DEMO_OBJ := $(COMMON_OBJ) $(CAN_COMMON_OBJ)

# Build rules
hand_demo.exe: hand_demo.cpp $(DEMO_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

hand_monitor.exe: hand_monitor.cpp $(DEMO_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

auto_detect.exe: auto_detect.cpp $(DEMO_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

hand_dfu.exe: hand_dfu.cpp $(DEMO_OBJ) $(DFU_COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(DFU_COMMON_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

# Default target
all: hand_demo.exe hand_monitor.exe auto_detect.exe hand_dfu.exe
	@echo "\033[1;32m[$(OS)] build done\033[0m"

# Run targets
run_%: %.exe
	@echo "\033[1;36m[Running] $<\033[0m"
	@$(RUN_ENV) ./$<

clean: clean-common
	rm -f *.exe
	rm -rf *.exe.dSYM

help:
	@echo "\033[1;32m=== Demo Examples ($(OS)) ===\033[0m"
	@echo "All examples auto-detect device and protocol by default."
	@echo ""
	@echo "Build:"
	@echo "  make                        # Build (default: ZQWL backend)"
	@echo "  make CAN_BACKEND=zlg        # Build with ZLG USB-CANFD"
	@echo "  make CAN_BACKEND=socketcan  # Build with SocketCAN (Linux)"
	@echo ""
	@echo "Run:"
	@echo "  ./hand_demo.exe             # Auto-detect all protocols"
	@echo "  ./hand_monitor.exe motor    # Auto-detect, motor monitor"
	@echo ""
	@echo "Manual init (optional, skip auto-detect):"
	@echo "  -m <port> <baud> <id>          Modbus"
	@echo "  -c <port> <baud> <id>          CAN 2.0 (ZQWL)"
	@echo "  -f <port> <arb> <data> <id>    CANFD (ZQWL)"
	@echo "  -s <iface> <id>                SocketCAN CAN 2.0 (Linux)"
	@echo "  -S <iface> <id>                SocketCAN CANFD (Linux)"
	@echo ""
	@echo "CAN Backend:"
	@echo "  ZQWL (default) - SDK built-in, auto-detect"
	@echo "  ZLG            - make CAN_BACKEND=zlg"
	@echo "  SocketCAN      - make CAN_BACKEND=socketcan (Linux)"

.PHONY: all clean help
