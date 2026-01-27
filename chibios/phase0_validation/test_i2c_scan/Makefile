##############################################################################
# RocketChip ChibiOS Phase 0 - I2C Bus Scan Test
# Uses software I2C (fallback driver) since hardware I2C is not implemented
# Output via debug probe UART
##############################################################################

##############################################################################
# Build global options
#

ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
endif

ifeq ($(USE_COPT),)
  USE_COPT =
endif

ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Disabled: LTO conflicts with portable make.exe on Windows
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# Disabled: Smart build uses Unix commands (cat/egrep) that fail on Windows
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = no
endif

##############################################################################
# Architecture specific options
#

ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x800
endif

ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

ifeq ($(USE_FPU),)
  USE_FPU = softfp
endif

ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-sp-d16
endif

##############################################################################
# Project, target, sources and paths
#

PROJECT = rocketchip_i2c_scan
MCU = cortex-m33

# Paths
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

# ChibiOS streams library for chprintf
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

# Linker script
LDSCRIPT = $(STARTUPLD)/RP2350_FLASH.ld

# C sources
# Include fallback software I2C driver instead of the broken I2Cv1 stub
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/hal/src/hal_safety.c \
       $(CHIBIOS)/os/hal/src/hal_buffered_serial.c \
       $(CHIBIOS)/os/hal/lib/fallback/I2C/hal_i2c_lld.c \
       main.c

CPPSRC = $(ALLCPPSRC)

ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Include directories
# Include fallback I2C headers
INCDIR = $(CONFDIR) $(ALLINC) $(CHIBIOS)/os/hal/lib/fallback/I2C

# Warnings
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User section
#

UDEFS = -DCRT0_VTOR_INIT=1 -DNDEBUG

UADEFS = -DCRT0_VTOR_INIT=1 -DCRT0_EXTRA_CORES_NUMBER=0

UINCDIR =
ULIBDIR =
ULIBS =

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

##############################################################################
# Custom rules - UF2 generation
#

PICOTOOL ?= picotool

$(BUILDDIR)/$(PROJECT).uf2: $(BUILDDIR)/$(PROJECT).elf
	$(PICOTOOL) uf2 convert $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).uf2

upload: $(BUILDDIR)/$(PROJECT).elf
	$(PICOTOOL) load -v -x $(BUILDDIR)/$(PROJECT).elf

uf2: $(BUILDDIR)/$(PROJECT).uf2

.PHONY: upload uf2
