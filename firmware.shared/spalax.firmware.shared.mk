SPALAX_FIRMWARE_SHARED_INCLUDES = \
	$(SPALAX_FIRMWARE_SHARED)/src \
	$(SPALAX_SHARED)/include \
	$(MESSAGING)/include \
	$(MESSAGING)/include/config

SPALAX_FIRMWARE_SHARED_LIB_DIR = $(SPALAX_FIRMWARE_SHARED)/build-$(PROJECT)/lib

ifeq ($(OS),Windows_NT)
	# This will print a warning if the directory exists - not sure how to suppress this
	# It will also return false stopping execution if using &&
    MAKE_DIR := mkdir build-$(PROJECT)
	MAKEFILES := "MinGW Makefiles"
	DEL_DIR := if exist build-$(PROJECT) rmdir /q /s build-$(PROJECT)
	# Ignore error on windows
	CONTINUE_EVEN_IF_ERROR_OPERATOR := &
else
    MAKE_DIR := mkdir -p build-$(PROJECT)
	MAKEFILES := "Unix Makefiles"
	DEL_DIR := rm -rf build-$(PROJECT)
	# We don't need the hacky error ignoring on non-windows systems
	CONTINUE_EVEN_IF_ERROR_OPERATOR := &&
endif

# We get the directory of the root makefile - so we can patch up relative paths
MAKEFILE_DIR := $(dir $(abspath $(firstword $(MAKEFILE_LIST))))

.PHONY: clean_spalax_shared build_spalax_shared

clean_spalax_shared:
	@cd $(SPALAX_FIRMWARE_SHARED) \
	&& $(DEL_DIR)

build_spalax_shared :
	@cd $(SPALAX_FIRMWARE_SHARED) \
	&& $(MAKE_DIR) \
	$(CONTINUE_EVEN_IF_ERROR_OPERATOR) cd build-$(PROJECT) \
	&& cmake \
		-G$(MAKEFILES) \
		-DCMAKE_TOOLCHAIN_FILE=Toolchain-arm-none-eabi.cmake \
		-DADDITIONAL_C_FLAGS="$(MCFLAGS) $(OPT) $(COPT) $(CWARN) -mthumb -mno-thumb-interwork $(subst -I, -I$(MAKEFILE_DIR),$(IINCDIR))" \
		.. \
	&& $(MAKE)


ifeq ($(BUILDDIR),)
  BUILDDIR = build
endif
ifeq ($(BUILDDIR),.)
  BUILDDIR = build
endif

# Every build must perform this to update the libraries
$(BUILDDIR)/$(PROJECT).elf : build_spalax_shared
