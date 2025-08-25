# Directories
CMSIS_ROOT_DIR = cmsis
CMSIS_CORE_DIR = $(CMSIS_ROOT_DIR)/core
CMSIS_DEVICE_DIR = $(CMSIS_ROOT_DIR)/device
CMSIS_DEVICE_INCLUDE_DIR = $(CMSIS_DEVICE_DIR)/Include

SRC_ROOT_DIR = src
APP_DIR = $(SRC_ROOT_DIR)/app
DRIVERS_DIR = $(SRC_ROOT_DIR)/drivers
MANAGERS_DIR = $(SRC_ROOT_DIR)/manager

INCLUDE_DIRS = $(CMSIS_DEVICE_INCLUDE_DIR) $(DRIVERS_DIR) $(CMSIS_CORE_DIR)
CPPCHECK_INCLUDE_DIRS = $(DRIVERS_DIR)

BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Toolchain
CC = arm-none-eabi-gcc
AS = $(CC) -x assembler-with-cpp
OBJCOPY = arm-none-eabi-objcopy
CPPCHECK = cppcheck

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

# Sources -seperate C and assembly files
C_SRC = \
	src/app/main.c \
	src/drivers/i2c_driver.c \
	src/drivers/spi_driver.c \
	src/drivers/usart_driver.c \
	src/managers/comms_manager.c \
	src/managers/logging_manager.c \
	src/managers/sensor_manager.c \
	tests/main_test.c

CMSIS_C_SRC = $(CMSIS_DEVICE_DIR)/system_stm32l4xx.c 
CMSIS_ASM_SRC = $(CMSIS_DEVICE_DIR)/startup_stm32l432xx.s

CPPCHECK_SRC = $(C_SRC)

# Objects - handle C and assembly separately
C_OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(C_SRC))
CMSIS_C_OBJ = $(OBJ_DIR)/cmsis/system_stm32l4xx.o
CMSIS_ASM_OBJ = $(OBJ_DIR)/cmsis/startup_stm32l432xx.o

OBJ = $(C_OBJ) $(CMSIS_C_OBJ) $(CMSIS_ASM_OBJ)

# Pattern rules
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(CFLAGS) -c $< -o $@

# Special rules for CMSIS files (absolute paths)
$(OBJ_DIR)/cmsis/system_stm32l4xx.o: $(CMSIS_C_SRC)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/cmsis/startup_stm32l432xx.o: $(CMSIS_ASM_SRC)
	@mkdir -p $(dir $@)
	$(AS) $(CFLAGS) -c $< -o $@

# Linking
$(ELF): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS)

# Binary image
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@


# Phonies
.PHONY: all clean flash cppcheck cppcheck-debug obj-debug setup-unity

all: $(ELF) $(BIN)

obj-deubg:
	@echo "C_OBJ"
	@echo $(C_OBJ)
	@echo "CMSIS_C_OBJ:"
	@echo $(CMSIS_C_OBJ)
	@echo "CMSIS_ASM_OBJ:"
	@echo $(CMSIS_ASM_OBJ)
	@echo "OBJ:"
	@echo $(OBJ)

clean:
	rm -rf $(BUILD_DIR)

flash: $(ELF)
	openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
		-c "program $< verify reset exit"

cppcheck:
	@$(CPPCHECK) --quiet --enable=all --error-exitcode=1 \
		--suppressions-list=cppcheck_suppressions.txt \
		--inline-suppr \
		--max-configs=20 \
		$(addprefix -I,$(INCLUDE_DIRS)) \
		$(CPPCHECK_SRC)

cppcheck-debug:
	@echo "CPPCHECK_SRC files:"
	@echo $(CPPCHECK_SRC)
	@echo "Include dirs:"
	@echo $(INCLUDE_DIRS)

setup-unity:
	@echo "Setting up unity ..."
	git clone https://github.com/ThrowTheSwitch/Unity.git unity
	@echo "Unity setup complete"