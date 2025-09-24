# Directories
CMSIS_ROOT_DIR = cmsis
CMSIS_CORE_DIR = $(CMSIS_ROOT_DIR)/core
CMSIS_DEVICE_DIR = $(CMSIS_ROOT_DIR)/device
CMSIS_DEVICE_INCLUDE_DIR = $(CMSIS_DEVICE_DIR)/Include

SRC_ROOT_DIR = src
APP_DIR = $(SRC_ROOT_DIR)/app
DRIVERS_DIR = $(SRC_ROOT_DIR)/drivers
MANAGERS_DIR = $(SRC_ROOT_DIR)/managers
TESTS_DIR = tests

UNITY_DIR = unity
UNITY_SRC_DIR = $(UNITY_DIR)/src

INCLUDE_DIRS = $(CMSIS_DEVICE_INCLUDE_DIR) $(APP_DIR) $(DRIVERS_DIR) $(MANAGERS_DIR) $(CMSIS_CORE_DIR)
CPPCHECK_INCLUDE_DIRS = $(DRIVERS_DIR) $(MANAGERS_DIR) $(APP_DIR)

BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
TEST_BUILD_DIR = $(BUILD_DIR)/tests

# Toolchain - Production
CC = arm-none-eabi-gcc
AS = $(CC) -x assembler-with-cpp
OBJCOPY = arm-none-eabi-objcopy
CPPCHECK = cppcheck

# Toolchain - Testing (native)
TEST_CC = gcc

# Platform detection for test execution
ifeq ($(OS),Windows_NT)
	TEST_EXE_EXT = .exe
	TEST_RUN_CMD =
else
	TEST_EXE_EXT = 
	TEST_RUN_CMD = ./
endif

# Files
TARGET = crisissense
TEST_TARGET = test_usart_driver

ELF = $(BUILD_DIR)/$(TARGET).elf
BIN = $(BUILD_DIR)/$(TARGET).bin
MAP = $(BUILD_DIR)/$(TARGET).map
TEST_EXE = $(TEST_BUILD_DIR)/$(TEST_TARGET)$(TEST_EXE_EXT)

LDSCRIPT = stm32l432kc.ld


# Flags
MCU = stm32l432kc
WFLAGS = -Wall -Wextra -Werror -Wshadow
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -ffreestanding -g $(WFLAGS)
CFLAGS += -DBME280_64BIT_ENABLE #Use 64-bit integers for sensor compensation in bme280
CFLAGS += $(addprefix -I,$(INCLUDE_DIRS)) -I.
LDFLAGS = -T $(LDSCRIPT) -lgcc -Wl,-Map=$(MAP) -nostdlib 

# Flags - Testing
TEST_WFLAGS = -Wall -Wextra -std=c99
TEST_CFLAGS = -g -O0 -DUNIT_TEST $(TEST_WFLAGS)
TEST_CFLAGS += -I$(DRIVERS_DIR) -I$(TEST_DIR) -I$(UNITY_DIR) -I.
TEST_LDFLAGS =


# Sources -seperate C and assembly files
C_SRC = \
	src/app/main.c \
	src/drivers/i2c_driver.c \
	src/drivers/spi_driver.c \
	src/drivers/usart_driver.c \
	src/drivers/dma_driver.c \
	src/drivers/rcc_driver.c \
	src/drivers/gpio_driver.c \
	src/drivers/bme280.c \
	src/managers/comms_manager.c \
	src/managers/logging_manager.c \
	src/managers/sensor_manager.c \
	src/managers/debugging_manager.c
	#tests/main_test.c

CMSIS_C_SRC = $(CMSIS_DEVICE_DIR)/system_stm32l4xx.c 
CMSIS_ASM_SRC = $(CMSIS_DEVICE_DIR)/startup_stm32l432xx.s

CPPCHECK_SRC = $(C_SRC)

# Sources - Testing
TEST_DRIVER_SRC = src/drivers/usart_driver.c 
TEST_SRC = tests/test_usart_driver.c 
UNITY_SRC = $(UNITY_SRC_DIR)/unity.c

# Objects - handle C and assembly separately
C_OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(C_SRC))
CMSIS_C_OBJ = $(OBJ_DIR)/cmsis/system_stm32l4xx.o
CMSIS_ASM_OBJ = $(OBJ_DIR)/cmsis/startup_stm32l432xx.o

OBJ = $(C_OBJ) $(CMSIS_C_OBJ) $(CMSIS_ASM_OBJ)

# Objects - Testing
TEST_DRIVER_OBJ = $(patsubst src/drivers/%.c,$(TEST_BUILD_DIR)/%.o,$(TEST_DRIVER_SRC))
TEST_OBJ = $(patsubst tests/%.c,$(TEST_BUILD_DIR)/%.o,$(TEST_SRC))
UNITY_OBJ = $(patsubst $(UNITY_SRC_DIR)/%.c,$(TEST_BUILD_DIR)/%.o,$(UNITY_SRC))

ALL_TEST_OBJ = $(TEST_DRIVER_OBJ) $(TEST_OBJ) $(UNITY_OBJ)

# Production Build Pattern rules
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

# Test build pattern rules
$(TEST_BUILD_DIR)/%.o: src/drivers/%.c
	@mkdir -p $(dir $@)
	$(TEST_CC) $(TEST_CFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/%.o: tests/%.c
	@mkdir -p $(dir $@)
	$(TEST_CC) $(TEST_CFLAGS) -c $< -o $@

$(TEST_BUILD_DIR)/%.o: $(UNITY_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(TEST_CC) $(TEST_CFLAGS) -c $< -o $@

# Linking - Production 
$(ELF): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS)

# Linking - Testing
$(TEST_EXE): $(ALL_TEST_OBJ)
	@mkdir -p $(dir $@)
	$(TEST_CC) $(TEST_CFLAGS) $(ALL_TEST_OBJ) -o $@ $(TEST_LDFLAGS)

# Binary image
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@


# Phonies
.PHONY: all clean flash cppcheck cppcheck-debug obj-debug setup-unity test test-build test-debug 

all: $(ELF) $(BIN)

test-build: $(TEST_EXE)

test: test-build
	@echo "Running Tests ..."
	$(TEST_RUN_CMD)$(TEST_EXE)

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
		-UUNIT_TEST \
		$(addprefix -I,$(INCLUDE_DIRS)) -I.\
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

# Debug targets for testing
test-debug:
	@echo "TEST_DRIVER_SRC:"
	@echo $(TEST_DRIVER_SRC)
	@echo "TEST_SRC:"
	@echo $(TEST_SRC)
	@echo "UNITY_SRC:"
	@echo $(UNITY_SRC)
	@echo "ALL_TEST_OBJ:"
	@echo $(ALL_TEST_OBJ)
	@echo "TEST_EXE:"
	@echo $(TEST_EXE)
	@echo "Platform: $(OS)"
	@echo "Test run command: '$(TEST_RUN_CMD)'"
