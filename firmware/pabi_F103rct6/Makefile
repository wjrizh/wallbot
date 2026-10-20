# STM32F103RCT6 GCC Makefile
# 用于在 Linux 下编译 Keil 移植项目（不含 Keil 专有语法时可行）

PROJECT = pabi_F103rct6
BUILD_DIR = build

# 工具链
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# 芯片与目标
CPU = -mcpu=cortex-m3
FPU =
FLOAT-ABI =

# 定义
C_DEFS = \
	-DUSE_HAL_DRIVER \
	-DSTM32F103xE

# 头文件搜索路径
C_INCLUDES = \
	-ICore/Inc \
	-ICore/Src \
	-IDrivers/STM32F1xx_HAL_Driver/Inc \
	-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
	-IDrivers/CMSIS/Include

# 编译选项
C_FLAGS = $(CPU) -mthumb -Wall -Wextra -Wno-unused-parameter \
	-Wno-missing-field-initializers -fdata-sections -ffunction-sections \
	-Og -g3 -std=gnu11 $(C_DEFS) $(C_INCLUDES)

AS_FLAGS = $(CPU) -mthumb -g3 -x assembler-with-cpp $(C_DEFS) $(C_INCLUDES)

LD_FLAGS = $(CPU) -mthumb -specs=nano.specs -specs=nosys.specs \
	-Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
	-Wl,-Tgcc_build/STM32F103RCTx_FLASH.ld

# 源文件
C_SOURCES = \
	Core/Src/main.c \
	Core/Src/gpio.c \
	Core/Src/spi.c \
	Core/Src/tim.c \
	Core/Src/usart.c \
	Core/Src/flash.c \
	Core/Src/mpu6050.c \
	Core/Src/inv_mpu.c \
	Core/Src/inv_mpu_dmp_motion_driver.c \
	Core/Src/stm32f1xx_hal_msp.c \
	Core/Src/stm32f1xx_it.c \
	Core/Src/system_stm32f1xx.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c

ASM_SOURCES = \
	Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xe.s

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

TARGET = $(BUILD_DIR)/$(PROJECT).elf
HEX = $(BUILD_DIR)/$(PROJECT).hex
BIN = $(BUILD_DIR)/$(PROJECT).bin

.PHONY: all clean

all: $(HEX) $(BIN)
	@$(SIZE) $(TARGET)

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(C_FLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(AS_FLAGS) -c $< -o $@

$(TARGET): $(OBJECTS)
	@echo "LD $@"
	@$(LD) $(LD_FLAGS) -o $@ $(OBJECTS) -lm

$(HEX): $(TARGET)
	@$(OBJCOPY) -O ihex $< $@
	@echo "HEX -> $@"

$(BIN): $(TARGET)
	@$(OBJCOPY) -O binary $< $@
	@echo "BIN -> $@"

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)