# -*- coding: utf-8 -*-
import os, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)
GCC_BIN = Path(r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin")
os.environ["PATH"] = str(GCC_BIN) + os.pathsep + r"C:\Windows\System32" + os.pathsep + r"C:\Windows"

def tool(name):
    p = GCC_BIN / (name + ".exe")
    if not p.exists():
        print("MISSING_TOOL", p)
        sys.exit(2)
    return str(p)

CC = tool("arm-none-eabi-gcc")
OBJCOPY = tool("arm-none-eabi-objcopy")
SIZE = tool("arm-none-eabi-size")

C_SOURCES = [
    "Core/Src/main.c",
    "Core/Src/gpio.c",
    "Core/Src/spi.c",
    "Core/Src/tim.c",
    "Core/Src/usart.c",
    "Core/Src/flash.c",
    "Core/Src/mpu6050.c",
    "Core/Src/inv_mpu.c",
    "Core/Src/inv_mpu_dmp_motion_driver.c",
    "Core/Src/stm32f1xx_hal_msp.c",
    "Core/Src/stm32f1xx_it.c",
    "Core/Src/system_stm32f1xx.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c",
]
ASM_SOURCES = [
    "Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xe.s",
]

C_FLAGS = [
    "-mcpu=cortex-m3", "-mthumb", "-Wall", "-Wextra", "-Wno-unused-parameter",
    "-Wno-missing-field-initializers", "-fdata-sections", "-ffunction-sections",
    "-Og", "-g3", "-std=gnu11",
    "-DUSE_HAL_DRIVER", "-DSTM32F103xE",
    "-ICore/Inc", "-ICore/Src",
    "-IDrivers/STM32F1xx_HAL_Driver/Inc",
    "-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy",
    "-IDrivers/CMSIS/Device/ST/STM32F1xx/Include",
    "-IDrivers/CMSIS/Include",
]
AS_FLAGS = [
    "-mcpu=cortex-m3", "-mthumb", "-g3", "-x", "assembler-with-cpp",
    "-DUSE_HAL_DRIVER", "-DSTM32F103xE",
    "-ICore/Inc", "-ICore/Src",
    "-IDrivers/STM32F1xx_HAL_Driver/Inc",
    "-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy",
    "-IDrivers/CMSIS/Device/ST/STM32F1xx/Include",
    "-IDrivers/CMSIS/Include",
]
BUILD = ROOT / "build"
BUILD.mkdir(exist_ok=True)
MAP = BUILD / "pabi_F103rct6.map"
LD_FLAGS = [
    "-mcpu=cortex-m3", "-mthumb", "-specs=nano.specs", "-specs=nosys.specs",
    "-Wl,--gc-sections", f"-Wl,-Map={MAP}",
    "-Wl,-Tgcc_build/STM32F103RCTx_FLASH.ld",
]

def run(cmd):
    print("+", " ".join(cmd), flush=True)
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit(r.returncode)

objs = []
for src in C_SOURCES:
    src_p = ROOT / src
    if not src_p.exists():
        print("MISSING", src_p)
        sys.exit(3)
    obj = BUILD / (Path(src).name.replace(".c", ".o"))
    print("CC", src, flush=True)
    run([CC] + C_FLAGS + ["-MMD", "-MP", "-c", str(src_p), "-o", str(obj)])
    objs.append(str(obj))

for src in ASM_SOURCES:
    src_p = ROOT / src
    if not src_p.exists():
        print("MISSING", src_p)
        sys.exit(3)
    obj = BUILD / (Path(src).name.replace(".s", ".o"))
    print("AS", src, flush=True)
    run([CC] + AS_FLAGS + ["-c", str(src_p), "-o", str(obj)])
    objs.append(str(obj))

elf = BUILD / "pabi_F103rct6.elf"
print("LD", elf, flush=True)
run([CC] + LD_FLAGS + ["-o", str(elf)] + objs + ["-lm"])
run([OBJCOPY, "-O", "ihex", str(elf), str(BUILD / "pabi_F103rct6.hex")])
print("HEX ->", BUILD / "pabi_F103rct6.hex", flush=True)
run([OBJCOPY, "-O", "binary", str(elf), str(BUILD / "pabi_F103rct6.bin")])
print("BIN ->", BUILD / "pabi_F103rct6.bin", flush=True)
run([SIZE, str(elf)])
print("BUILD_OK", flush=True)
