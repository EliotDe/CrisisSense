# Directories
CMSIS_ROOT_DIR = cmsis
CMSIS_CORE_DIR = $(CMSIS_ROOT_DIR)/core
CMSIS_DEVICE_DIR = $(CMSIS_ROOT_DIR)/device
CMSIS_DEVICE_INCLUDE_DIR = $(CMSIS_DEVICE_DIR)/Include

SRC_ROOT_DIR = src
APP_DIR = $(SRC_ROOT_DIR)app
DRIVERS_DIR = $(SRC_ROOT_DIR)drivers
MANAGERS_DIR = $(SRC_ROOT_DIR)managers

INCLUDE_DIRS = $(CMSIS_DEVICE_INCLUDE_DIR) $(DRIVERS_DIR) $(CMSIS_CORE_DIR)

BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Toolchain
CC = arm-none-eabi-gcc
AS = $(CC) -x assembler-with-cpp
OBJCOPY = arm-none-eabi-objcopy

# Files
TARGET = crisissense
ELF = $(BUILD_DIR)/$(TARGET).elf
BIN = $(BUILD_DIR)/$(TARGET).bin
MAP = $(BUILD_DIR)/$(TARGET).map
LDSCRIPT = stm32l432kc.ld

# Flags
MCU = stm32l432kc
WFLAGS = -Wall -Wextra -Werror -Wshadow
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -ffreestanding -g $(WFLAGS)
CFLAGS += $(addprefix -I,$(INCLUDE_DIRS))
LDFLAGS = -T $(LDSCRIPT) -nostdlib -Wl,-Map=$(MAP)

# Sources
SRC = \
	src/app/main.c \
	cmsis/device/system_stm32l4xx.c \
	cmsis/device/startup_stm32l432xx.s \
	src/drivers/i2c_driver.c \
	src/drivers/spi_driver.c \
	src/drivers/usart_driver.c \
	src/managers/comms_manager.c \
	src/managers/logging_manager.c \
	src/managers/sensor_manager.c \
	tests/tests/main_test.c

# Objects (mirror src paths inside OBJ_DIR)
OBJ = $(patsubst %,$(OBJ_DIR)/%,$(SRC:.c=.o))
OBJ := $(OBJ:.s=.o)

# Pattern rules
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(CFLAGS) -c $< -o $@

# Linking
$(ELF): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Binary image
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

# Phonies
.PHONY: all clean flash

all: $(ELF) $(BIN)

clean:
	rm -rf $(BUILD_DIR)

flash: $(BIN)
	openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
		-c "program $< verify reset exit"
