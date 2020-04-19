SCARD_IO_MOD_DIR := $(USERMOD_DIR)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(SCARD_IO_MOD_DIR)/scard_io.c
SRC_USERMOD += $(SCARD_IO_MOD_DIR)/t1_protocol/t1_protocol.c

# We can add our module folder to include paths if needed
CFLAGS_USERMOD += -I$(SCARD_IO_MOD_DIR)/t1_protocol