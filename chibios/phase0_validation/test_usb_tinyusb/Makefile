##############################################################################
# RocketChip ChibiOS Phase 0 - USB CDC TinyUSB Test
# Uses TinyUSB library with Pico SDK hardware headers
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

ifeq ($(USE_LTO),)
  USE_LTO = no
endif

ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

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

PROJECT = rocketchip_usb_tinyusb
MCU = cortex-m33

# ChibiOS path
CHIBIOS  := ../../ChibiOS
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Pico SDK and TinyUSB paths
PICO_SDK := $(HOME)/.pico-sdk/sdk/2.2.0
TINYUSB  := $(PICO_SDK)/lib/tinyusb

# Include ChibiOS build components
include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/RP/RP2350/platform.mk
include $(CHIBIOS)/os/hal/boards/RP_PICO2_RP2350/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv8-M-ML/compilers/GCC/mk/port.mk

# Include streams library for chsnprintf
include $(CHIBIOS)/os/hal/lib/streams/streams.mk

# Linker script
LDSCRIPT = $(STARTUPLD)/RP2350_FLASH.ld

# TinyUSB source files
TINYUSB_SRC = \
    $(TINYUSB)/src/tusb.c \
    $(TINYUSB)/src/common/tusb_fifo.c \
    $(TINYUSB)/src/device/usbd.c \
    $(TINYUSB)/src/device/usbd_control.c \
    $(TINYUSB)/src/class/cdc/cdc_device.c \
    $(TINYUSB)/src/portable/raspberrypi/rp2040/dcd_rp2040.c \
    $(TINYUSB)/src/portable/raspberrypi/rp2040/rp2040_usb.c

# Pico SDK source files - NONE
# All Pico SDK runtime functions are stubbed in pico_stubs.c to avoid
# conflicts with ChibiOS initialization and interrupt handling.
PICO_SDK_SRC =

# Pico SDK assembly files - REMOVED irq_handler_chain.S as it conflicts with ChibiOS vector table
# We use pico_stubs.c for interrupt routing instead
PICO_SDK_ASM =

# C sources
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/hal/src/hal_safety.c \
       $(CHIBIOS)/os/hal/src/hal_buffered_serial.c \
       $(TINYUSB_SRC) \
       $(PICO_SDK_SRC) \
       usb_descriptors.c \
       pico_stubs.c \
       main.c

# C++ sources
CPPSRC = $(ALLCPPSRC)

# ASM sources
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# TinyUSB include paths
TINYUSB_INC = \
    $(TINYUSB)/src \
    $(TINYUSB)/src/common \
    $(TINYUSB)/src/device \
    $(TINYUSB)/src/class/cdc \
    $(TINYUSB)/src/portable/raspberrypi/rp2040

# Pico SDK shim (CMake-generated headers we must provide)
# IMPORTANT: Must come first to provide version.h, config_autogen.h
PICO_SHIM_INC = ./pico_shim

# Pico SDK include paths (comprehensive set for TinyUSB)
# Note: SDK 2.2.0 has highly fragmented include structure
PICO_SDK_INC = \
    $(PICO_SDK)/src/common/pico_base_headers/include \
    $(PICO_SDK)/src/boards/include \
    $(PICO_SDK)/src/common/hardware_claim/include \
    $(PICO_SDK)/src/common/pico_binary_info/include \
    $(PICO_SDK)/src/common/pico_sync/include \
    $(PICO_SDK)/src/common/pico_time/include \
    $(PICO_SDK)/src/common/pico_util/include \
    $(PICO_SDK)/src/rp2350/hardware_regs/include \
    $(PICO_SDK)/src/rp2350/hardware_structs/include \
    $(PICO_SDK)/src/rp2350/pico_platform/include \
    $(PICO_SDK)/src/rp2_common/boot_bootrom_headers/include \
    $(PICO_SDK)/src/rp2_common/hardware_boot_lock/include \
    $(PICO_SDK)/src/rp2_common/hardware_base/include \
    $(PICO_SDK)/src/rp2_common/hardware_clocks/include \
    $(PICO_SDK)/src/rp2_common/hardware_gpio/include \
    $(PICO_SDK)/src/rp2_common/hardware_irq/include \
    $(PICO_SDK)/src/rp2_common/hardware_pll/include \
    $(PICO_SDK)/src/rp2_common/hardware_resets/include \
    $(PICO_SDK)/src/rp2_common/hardware_sync/include \
    $(PICO_SDK)/src/rp2_common/hardware_sync_spin_lock/include \
    $(PICO_SDK)/src/rp2_common/hardware_ticks/include \
    $(PICO_SDK)/src/rp2_common/hardware_timer/include \
    $(PICO_SDK)/src/rp2_common/hardware_xosc/include \
    $(PICO_SDK)/src/rp2_common/pico_atomic/include \
    $(PICO_SDK)/src/rp2_common/pico_bootrom/include \
    $(PICO_SDK)/src/rp2_common/pico_platform_common/include \
    $(PICO_SDK)/src/rp2_common/pico_platform_compiler/include \
    $(PICO_SDK)/src/rp2_common/pico_platform_panic/include \
    $(PICO_SDK)/src/rp2_common/pico_platform_sections/include \
    $(PICO_SDK)/src/rp2_common/pico_runtime/include \
    $(PICO_SDK)/src/rp2_common/pico_runtime_init/include \
    $(PICO_SDK)/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include

# Include directories
# Note: PICO_SHIM_INC must come FIRST to override CMake-generated headers
INCDIR = $(PICO_SHIM_INC) $(CONFDIR) $(ALLINC) . $(TINYUSB_INC) $(PICO_SDK_INC)

# C warnings
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Wno-unused-parameter

# C++ warnings
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User section
#

# User C defines - configure for RP2350 and TinyUSB
UDEFS = -DCRT0_VTOR_INIT=1 -DNDEBUG \
        -DPICO_RP2350=1 \
        -DPICO_RP2040=0 \
        -DLIB_PICO_PLATFORM=1 \
        -DLIB_PICO_SYNC=1 \
        -DLIB_PICO_TIME=1 \
        -DCFG_TUSB_MCU=OPT_MCU_RP2040 \
        -DTUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX=0

# User ASM defines
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
# Custom rules
#

PICOTOOL ?= picotool

$(BUILDDIR)/$(PROJECT).uf2: $(BUILDDIR)/$(PROJECT).elf
	$(PICOTOOL) uf2 convert $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).uf2

upload: $(BUILDDIR)/$(PROJECT).uf2
	$(PICOTOOL) load -v -x $(BUILDDIR)/$(PROJECT).uf2

uf2: $(BUILDDIR)/$(PROJECT).uf2

.PHONY: upload uf2
