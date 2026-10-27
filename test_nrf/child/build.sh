#!/bin/sh
# 用现有 pabi_F103rct6 工程的 Drivers/Core 编译本测试固件
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
PRJ="$ROOT/firmware/pabi_F103rct6"
WORK="/tmp/nrf_test_build"

rm -rf "$WORK"
mkdir -p "$WORK"
cp -a "$PRJ/Drivers" "$PRJ/gcc_build" "$PRJ/Core" "$WORK/"
cp "$PRJ/Makefile" "$WORK/"
cp "$HERE/main_test.c" "$WORK/Core/Src/main.c"
cp "$HERE/adj_plan.h" "$WORK/Core/Src/adj_plan.h"

cd "$WORK" && make -j4

echo ""
echo "BIN: $WORK/build/pabi_F103rct6.bin"
echo "烧录: st-flash --reset write $WORK/build/pabi_F103rct6.bin 0x8000000"
