# Demo Examples - Cross-platform Makefile
# Auto-detect protocol and device type
.DEFAULT_GOAL := all

include ../common.mk

LIBS := $(BASE_LIBS)

ifeq ($(OS),linux)
    EXTRA_LDFLAGS := -rdynamic -ldl
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

hand_protolbuf_test.exe: hand_protolbuf_test.cpp $(DEMO_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

debug_detect.exe: debug_detect.cpp $(DEMO_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $< $(DEMO_OBJ) $(INCLUDE) $(LIBS) $(EXTRA_LDFLAGS)
	@echo "\033[1;32m[$(OS)] Built $@\033[0m"

# Default target
all: hand_demo.exe hand_monitor.exe auto_detect.exe hand_dfu.exe hand_protolbuf_test.exe debug_detect.exe
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
	@echo "  make                        # All CAN backends compiled"
	@echo "  make STARK_NO_CAN=1         # Disable CAN support"
	@echo ""
	@echo "Run:"
	@echo "  ./hand_demo.exe             # Auto-detect (default: SocketCAN)"
	@echo "  ./hand_monitor.exe motor    # Auto-detect, motor monitor"
	@echo ""
	@echo "Runtime CAN backend selection:"
	@echo "  STARK_CAN_BACKEND=zlg ./hand_demo.exe    # Use ZLG adapter"
	@echo "  STARK_CAN_BACKEND=socketcan ./hand_demo.exe  # Use SocketCAN"
	@echo ""
	@echo "Manual init (optional, skip auto-detect):"
	@echo "  -m <port> <baud> <id>          Modbus"
	@echo "  -c <port> <baud> <id>          CAN 2.0 (ZQWL)"
	@echo "  -f <port> <arb> <data> <id>    CANFD (ZQWL)"
	@echo "  -s <iface> <id>                SocketCAN CAN 2.0"
	@echo "  -S <iface> <id>                SocketCAN CANFD"
	@echo "  -z <id>                        ZLG CAN 2.0"
	@echo "  -Z <id>                        ZLG CANFD"

.PHONY: all clean help
