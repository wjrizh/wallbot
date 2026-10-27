@echo off
rem 修正版烧录脚本: OpenOCD 的 TCL 会吃掉反斜杠转义(\b退格等)且中文路径易出问题,
rem 因此先把 hex 拷到 ASCII 短路径 C:\Temp\pabi_flash, 再用正斜杠烧录。
rem 用法: _flash_fixed.bat  (或双击)
setlocal
set OCD=C:\Users\Administrator\AppData\Local\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe\xpack-openocd-0.12.0-7\bin\openocd.exe
if not exist C:\Temp\pabi_flash mkdir C:\Temp\pabi_flash
copy /y "%~dp0build\pabi_F103rct6.hex" C:\Temp\pabi_flash\pabi_F103rct6.hex
echo HEX=C:/Temp/pabi_flash/pabi_F103rct6.hex
"%OCD%" -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program C:/Temp/pabi_flash/pabi_F103rct6.hex verify reset exit"
echo EXIT=%ERRORLEVEL%
endlocal