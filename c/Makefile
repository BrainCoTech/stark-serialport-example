# Top-level Makefile for C/C++ examples
# Main entry point: demo/

.DEFAULT_GOAL := all

# Pass through variables to sub-makefiles
MAKE_ARGS := $(if $(TARGET),TARGET=$(TARGET)) $(if $(CAN_BACKEND),CAN_BACKEND=$(CAN_BACKEND))

# Build demo (main entry point)
all: demo

demo:
	$(MAKE) -C demo $(MAKE_ARGS)

# Clean
clean:
	$(MAKE) -C demo clean
	@echo "\033[1;32m=== Cleaned ===\033[0m"

# Run targets (with underscore)
run_hand_%:
	@$(MAKE) -C demo $(MAKE_ARGS) $@

run_auto_%:
	@$(MAKE) -C demo $(MAKE_ARGS) $@

# Run targets (without underscore, e.g., "make run hand_demo")
run:
	@if [ -z "$(filter-out run,$(MAKECMDGOALS))" ]; then \
		echo "Usage: make run <target>"; \
		echo "  make run hand_demo"; \
		echo "  make run hand_monitor"; \
		echo "  make run hand_dfu"; \
		echo "  make run auto_detect"; \
	else \
		$(MAKE) -C demo $(MAKE_ARGS) run_$(filter-out run,$(MAKECMDGOALS)); \
	fi

# Catch-all for run arguments (hand_demo, hand_monitor, etc.)
hand_demo hand_monitor hand_dfu auto_detect:
	@:

# Help
help:
	@echo "\033[1;32m=== Stark C/C++ Examples ===\033[0m"
	@echo ""
	@echo "\033[1;33mBuild:\033[0m"
	@echo "  make                        # Build (default: SDK built-in ZQWL only)"
	@echo "  make TARGET=win             # Cross-compile for Windows"
	@echo "  make CAN_BACKEND=zlg        # Enable ZLG support (Linux/Windows)"
	@echo "  make CAN_BACKEND=socketcan  # Enable SocketCAN support (Linux only)"
	@echo ""
	@echo "\033[1;33mRun:\033[0m"
	@echo "  make run hand_demo          # Comprehensive demo (8 modes)"
	@echo "  make run hand_monitor       # Real-time data monitor"
	@echo "  make run hand_dfu           # Firmware upgrade"
	@echo "  make run auto_detect        # Device detection"
	@echo ""
	@echo "  (Alternative syntax with underscore also works:)"
	@echo "  make run_hand_demo"
	@echo ""
	@echo "\033[1;33mClean:\033[0m"
	@echo "  make clean                  # Clean all"
	@echo ""
	@echo "\033[1;33mMore info:\033[0m"
	@echo "  See demo/README.md for detailed usage"

.PHONY: all demo clean help run hand_demo hand_monitor hand_dfu auto_detect
