################################################################################
# EtherCAT REVO2 Makefile
# Independent EtherCAT library for REVO2 hand control
################################################################################

.DEFAULT_GOAL := all

################################################################################
# Configuration
################################################################################

# Compiler settings
CC := gcc
CXX := g++
CFLAGS := -g -Wall
CXXFLAGS := -g -Wall -std=c++17

# Paths and libraries
INCLUDE := -I/usr/local/include
COMMON_LIBS := -L/usr/local/lib -Wl,-rpath,/usr/local/lib:/usr/lib -lethercat

# Library modules
LIB_MODULES := ec_sdo ec_app ec_feedback ec_utils ec_pdo ec_control ec_demo_helpers
LIB_SOURCES := $(addsuffix .cpp,$(LIB_MODULES))
COMMON_OBJ := $(addsuffix .o,$(LIB_MODULES))

# Example programs (exclude library modules)
BINARIES := $(basename $(filter-out $(LIB_SOURCES),$(wildcard *.cpp)))

################################################################################
# Build Rules - Library Modules
################################################################################

# Pattern rule for library object files
ec_%.o: ec_%.cpp ec_%.h
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDE)
	@echo "\033[1;32m[Built] $@\033[0m"

# Dependencies for library modules (header dependencies only)
ec_sdo.o: ec_types.h ec_constants.h
ec_app.o: ec_common.h ec_sdo.h
ec_feedback.o: ec_common.h ec_types.h
ec_utils.o: ec_types.h
ec_pdo.o: ec_types.h ec_constants.h
ec_control.o: ec_types.h ec_constants.h ec_utils.h
ec_demo_helpers.o: ec_control.h ec_feedback.h ec_utils.h

################################################################################
# Build Rules - Executables
################################################################################

# SDO programs (no capabilities needed)
%sdo: %sdo.cpp $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@.exe $< $(COMMON_OBJ) \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"
	@echo "\033[1;32m[SDO] No capabilities needed for SDO operations\033[0m"

# PDO programs (require capabilities for realtime and network access)
%_pdo: %_pdo.cpp $(COMMON_OBJ)
	$(CXX) $(CXXFLAGS) -o $@.exe $< $(COMMON_OBJ) \
		$(INCLUDE) \
		$(COMMON_LIBS)
	@echo "\033[1;33m[linux] build $@.cpp done\033[0m"
	@echo "\033[1;36m[Setting capabilities] Granting network and realtime permissions...\033[0m"
	@sudo setcap cap_sys_nice,cap_net_raw,cap_net_admin=eip $@.exe || echo "Warning: setcap failed, you may need to run with sudo"

################################################################################
# Main Targets
################################################################################

all: $(BINARIES)

clean:
	rm -f $(addsuffix .exe,$(BINARIES)) $(COMMON_OBJ)

################################################################################
# Run Targets
################################################################################

.PHONY: run_revo2_pdo run_revo2_sdo run_revo2_touch_pdo run_revo2_touch_pressure_pdo run_revo2_touch_sdo run_revo2_touch_pressure_sdo

# Pattern rule for run targets
run_%: %.exe
	@echo "\033[1;36m[Running] $<\033[0m"
	@./$<

# Ensure executable exists before running
%.exe:
	@echo "\033[1;31m[ERROR] $@ not found. Please build first: make $(basename $@)\033[0m"
	@exit 1

# Short aliases
.PHONY: run_pdo run_sdo run_touch_pdo run_touch_sdo run_touch_pressure_pdo run_touch_pressure_sdo
run_pdo: run_revo2_pdo
run_sdo: run_revo2_sdo
run_touch_pdo: run_revo2_touch_pdo
run_touch_sdo: run_revo2_touch_sdo
run_touch_pressure_pdo: run_revo2_touch_pressure_pdo
run_touch_pressure_sdo: run_revo2_touch_pressure_sdo

# Generic run target
.PHONY: run
run:
	@target="$(filter-out run,$(MAKECMDGOALS))"; \
	if [ -z "$$target" ]; then \
		echo "\033[1;31m[ERROR] Usage: make run <target>\033[0m"; \
		echo "Examples:"; \
		echo "  make run revo2_pdo"; \
		echo "  make run revo2_sdo"; \
		echo "  make run revo2_touch_pdo"; \
		exit 1; \
	else \
		if [ ! -f "$$target.exe" ]; then \
			echo "\033[1;33m[Building] $$target first...\033[0m"; \
			$(MAKE) $$target; \
		fi; \
		echo "\033[1;36m[Running] $$target.exe\033[0m"; \
		./$$target.exe; \
	fi

# Prevent make from building targets when used with 'run'
ifneq ($(filter run,$(MAKECMDGOALS)),)
$(filter-out run run_%,$(MAKECMDGOALS)):
	@:
endif

################################################################################
# Utility Targets
################################################################################

# Kill all EtherCAT processes
.PHONY: kill killall
kill killall:
	@echo "\033[1;33m[Killing] EtherCAT processes...\033[0m"
	@sudo fuser -k /dev/EtherCAT0 2>/dev/null || echo "No processes to kill"
	@echo "\033[1;32m[Done] EtherCAT processes terminated\033[0m"

# Restart EtherCAT service
.PHONY: restart_ethercat
restart_ethercat:
	@echo "\033[1;33m[Restarting] EtherCAT service...\033[0m"
	sudo systemctl restart ethercat
	@echo "\033[1;32m[Done] EtherCAT service restarted\033[0m"

################################################################################
# Help
################################################################################
.PHONY: help
help:
	@echo "\033[1;32m=== EtherCAT Library Makefile ===\033[0m"
	@echo ""
	@echo "\033[1;33mBuild targets:\033[0m"
	@echo "  make              - Build all executables"
	@echo "  make revo2_pdo    - Build PDO example"
	@echo "  make revo2_sdo    - Build SDO example"
	@echo "  make clean        - Clean all build files"
	@echo ""
	@echo "\033[1;33mUtility targets:\033[0m"
	@echo "  make kill                - Kill all EtherCAT processes"
	@echo "  make restart_ethercat    - Restart EtherCAT service"
	@echo ""
	@echo "\033[1;33mRun targets:\033[0m"
	@echo "  make run <target>                 - Generic run command"
	@echo "  make run revo2_pdo                - Run PDO example"
	@echo "  make run revo2_sdo                - Run SDO example"
	@echo ""
	@echo "  make run_revo2_pdo                - Run PDO example"
	@echo "  make run_revo2_sdo                - Run SDO example"
	@echo "  make run_revo2_touch_pdo          - Run touch PDO example"
	@echo "  make run_revo2_touch_pressure_pdo - Run touch pressure PDO example"
	@echo "  make run_revo2_touch_sdo          - Run touch SDO example"
	@echo "  make run_revo2_touch_pressure_sdo - Run touch pressure SDO example"
	@echo ""
	@echo "\033[1;33mShort aliases:\033[0m"
	@echo "  make run_pdo                      - Same as run_revo2_pdo"
	@echo "  make run_sdo                      - Same as run_revo2_sdo"
	@echo "  make run_touch_pdo                - Same as run_revo2_touch_pdo"
	@echo "  make run_touch_sdo                - Same as run_revo2_touch_sdo"
	@echo ""
	@echo "\033[1;33mAvailable executables:\033[0m"
	@echo "  $(BINARIES)"

################################################################################
# Phony Targets Declaration
################################################################################

.PHONY: all clean run help kill killall restart_ethercat
