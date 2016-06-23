AVIONICS_INCLUDES = \
	$(AVIONICS)/include \
	$(AVIONICS)/messaging/include \
	$(AVIONICS)/messaging/include/config

AVIONICS_LIB_DIR = $(AVIONICS)/build-fw $(AVIONICS)/build-fw/messaging
AVIONICS_LIB = $(AVIONICS_LIB_DIR)/libAvionics.a $(AVIONICS_LIB_DIR)/messaging/libmessaging.a

ifeq ($(OS),Windows_NT)
	# This will print a warning if the directory exists - not sure how to suppress this
	# It will also return false stopping execution if using &&
    MAKE_DIR := mkdir build-fw
	MAKEFILES := "MinGW Makefiles"
	DEL_DIR := if exist build-fw rmdir /q /s build-fw
	# Ignore error on windows
	CONTINUE_EVEN_IF_ERROR_OPERATOR := &
else
    MAKE_DIR := mkdir -p build-fw
	MAKEFILES := "Unix Makefiles"
	DEL_DIR := rm -rf build-fw
	# We don't need the hacky error ignoring on non-windows systems
	CONTINUE_EVEN_IF_ERROR_OPERATOR := &&
endif

# We get the directory of the root makefile - so we can patch up relative paths
MAKEFILE_DIR := $(dir $(abspath $(firstword $(MAKEFILE_LIST))))

.PHONY: clean_avionics build_avionics

build_avionics:
	@ cd $(AVIONICS) \
	&& $(MAKE_DIR) \
	$(CONTINUE_EVEN_IF_ERROR_OPERATOR) cd build-fw \
	&& cmake \
		-G$(MAKEFILES) \
		-DCMAKE_TOOLCHAIN_FILE=Toolchain-arm-none-eabi.cmake \
		-DAVIONICS_OS=chibios \
		-DADDITIONAL_C_FLAGS="$(MCFLAGS) $(OPT) $(COPT) $(CWARN) -mthumb -mno-thumb-interwork $(subst -I, -I$(MAKEFILE_DIR),$(IINCDIR))" \
		.. \
	&& $(MAKE)

all : build_avionics $(OBJS) $(OUTFILES) MAKE_ALL_RULE_HOOK


clean_avionics:
	@cd $(AVIONICS) \
	&& $(DEL_DIR)
