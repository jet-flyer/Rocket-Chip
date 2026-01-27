##############################################################################
# RocketChip ChibiOS Phase 0 - LED Blink Test
# Based on ChibiOS RT-RP2350-BLINK demo
# Modified for: dual-core enabled, Adafruit Feather RP2350
##############################################################################

##############################################################################
# Build global options
#

# Compiler options
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable linker garbage collection
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable link time optimizations (LTO)
# Disabled: LTO conflicts with portable make.exe on Windows
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# Verbose compile output
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# Smart build (only compile used modules)
# Disabled: Smart build uses Unix commands (cat/egrep) that fail on Windows
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = no
endif

##############################################################################
# Architecture specific options
#

# Process stack size (main thread)
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Exception stack size
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# FPU mode (no, softfp, hard)
ifeq ($(USE_FPU),)
  USE_FPU = softfp
endif

# FPU options for Cortex-M33
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16
endif

##############################################################################
# Project, target, sources and paths
#

# Project name
PROJECT = rocketchip_blink

# Target MCU
MCU = cortex-m33

# ChibiOS path (relative to this directory)
CHIBIOS  := ../../ChibiOS
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Include ChibiOS build components
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/RP/RP2350/platform.mk
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML/compilers/GCC/mk/port.mk

# Linker script - Flash execution (XIP)
LDSCRIPT = $(STARTUPLD)/RP2350_FLASH.ld

# C sources
# Note: hal_safety.c is missing from ChibiOS hal.mk when USE_SMART_BUILD=no
# We add it manually here to fix undefined references to halRegWaitAnySet32X/halSftFail
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/hal/src/hal_safety.c \
       $(CHIBIOS)/os/hal/src/hal_buffered_serial.c \
       main.c

# C++ sources
CPPSRC = $(ALLCPPSRC)

# ASM sources
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include directories
INCDIR = $(CONFDIR) $(ALLINC)

# C warnings
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# C++ warnings
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User section
#

# User C defines
# DUAL-CORE: Remove single-core restrictions
UDEFS = -DCRT0_VTOR_INIT=1 -DNDEBUG

# User ASM defines
# Single-core for now (CRT0_EXTRA_CORES_NUMBER=0), enable Core1 later
UADEFS = -DCRT0_VTOR_INIT=1 -DCRT0_EXTRA_CORES_NUMBER=0

# User include directories
UINCDIR =

# User library directories
ULIBDIR =

# User libraries
ULIBS =

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

##############################################################################
# Custom rules - UF2 generation and upload
#

PICOTOOL ?= picotool

# Generate UF2 from ELF
$(BUILDDIR)/$(PROJECT).uf2: $(BUILDDIR)/$(PROJECT).elf
	$(PICOTOOL) uf2 convert $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).uf2

# Upload via picotool
upload: $(BUILDDIR)/$(PROJECT).uf2
	$(PICOTOOL) load -v -x $(BUILDDIR)/$(PROJECT).uf2

# Convenience target
uf2: $(BUILDDIR)/$(PROJECT).uf2

.PHONY: upload uf2
