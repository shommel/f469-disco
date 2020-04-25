SCARD_IO_MOD_DIR := $(USERMOD_DIR)

SCARD_DEBUG := 1 #!!!!!

# Reproduce environment variables from main makefile (not defined at the moment)
SCARD_MPY_DIR = $(USERMOD_DIR)/../../micropython
SCARD_MCU_SERIES_UPPER = $(shell echo $(MCU_SERIES) | tr '[:lower:]' '[:upper:]')
SCARD_HAL_DIR = $(SCARD_MPY_DIR)/lib/stm32lib/STM32$(SCARD_MCU_SERIES_UPPER)xx_HAL_Driver

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(SCARD_IO_MOD_DIR)/scard_io.c
SRC_USERMOD += $(SCARD_IO_MOD_DIR)/util/scard_util.c
SRC_USERMOD += $(SCARD_IO_MOD_DIR)/t1_protocol/t1_protocol.c

# Add hal_smartcard.c from HAL because it is not included in MicroPython build
SRC_USERMOD += $(SCARD_HAL_DIR)/Src/stm32$(MCU_SERIES)xx_hal_smartcard.c

# We can add our module folder to include paths if needed
CFLAGS_USERMOD += -I$(SCARD_IO_MOD_DIR)/t1_protocol
