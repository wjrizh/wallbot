@echo off
setlocal
set "PATH=C:\Windows\System32;C:\Windows;C:\Windows\System32\Wbem;C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin;D:\ProgramData\Anaconda3;D:\ProgramData\Anaconda3\Scripts"
cd /d "%~dp0"
echo === build start ===
where arm-none-eabi-gcc
where python
python _build_with_python.py > build_log_merge.txt 2>&1
set EC=%ERRORLEVEL%
echo EXITCODE=%EC%
type build_log_merge.txt
exit /b %EC%
