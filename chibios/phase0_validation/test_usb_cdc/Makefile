##############################################################################
# RocketChip ChibiOS Phase 0 - USB CDC Test
# ChibiOS + TinyUSB integration for USB serial debugging
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

PROJECT = rocketchip_usb_cdc
MCU = cortex-m33

# Paths
CHIBIOS  := ../../ChibiOS
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Pico SDK path for TinyUSB
PICO_SDK := $(HOME)/.pico-sdk/sdk/2.2.0
TINYUSB  := $(PICO_SDK)/lib/tinyusb/src

# Include ChibiOS build components
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/RP/RP2350/platform.mk
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML/compilers/GCC/mk/port.mk

# Linker script
LDSCRIPT = $(STARTUPLD)/RP2350_FLASH.ld

# TinyUSB sources
TINYUSBSRC = $(TINYUSB)/tusb.c \
             $(TINYUSB)/common/tusb_fifo.c \
             $(TINYUSB)/device/usbd.c \
             $(TINYUSB)/device/usbd_control.c \
             $(TINYUSB)/class/cdc/cdc_device.c \
             $(TINYUSB)/portable/raspberrypi/rp2040/dcd_rp2040.c \
             $(TINYUSB)/portable/raspberrypi/rp2040/rp2040_usb.c

# C sources
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/hal/src/hal_safety.c \
       $(CHIBIOS)/os/hal/src/hal_buffered_serial.c \
       $(TINYUSBSRC) \
       usb_descriptors.c \
       main.c

CPPSRC = $(ALLCPPSRC)

ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# TinyUSB include directories
# Pico stubs directory first to provide missing generated headers
TINYUSBINC = ./pico_stubs \
             $(TINYUSB) \
             $(PICO_SDK)/src/rp2_common/hardware_resets/include \
             $(PICO_SDK)/src/rp2_common/hardware_irq/include \
             $(PICO_SDK)/src/rp2_common/hardware_sync/include \
             $(PICO_SDK)/src/rp2_common/hardware_base/include \
             $(PICO_SDK)/src/rp2350/hardware_regs/include \
             $(PICO_SDK)/src/rp2350/hardware_structs/include \
             $(PICO_SDK)/src/common/pico_base_headers/include \
             $(PICO_SDK)/src/rp2350/pico_platform/include \
             $(PICO_SDK)/src/common/pico_sync/include \
             $(PICO_SDK)/src/common/pico_time/include

INCDIR = $(CONFDIR) $(ALLINC) $(TINYUSBINC)

# Warnings
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User section
#

# Defines for TinyUSB
UDEFS = -DCRT0_VTOR_INIT=1 -DNDEBUG \
        -DCFG_TUSB_MCU=OPT_MCU_RP2040 \
        -DPICO_RP2350=1 \
        -DLIB_PICO_PLATFORM_PANIC=1

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
