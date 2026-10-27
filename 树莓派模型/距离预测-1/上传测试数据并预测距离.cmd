@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
if "%~1"=="" (
  echo 请将一个 .pridb 文件或包含 .pridb/.tradb 的文件夹拖到本文件上。
  echo 也可以在命令行运行：
  echo node predict_distance_10.20.js "D:\路径\测试数据.pridb"
  pause
  exit /b 2
)
set "NODE=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
if not exist "%NODE%" (
  where node >nul 2>nul
  if errorlevel 1 (
    echo 未找到 Node.js。请安装 Node.js 22 或更新版本后重试。
    pause
    exit /b 1
  )
  set "NODE=node"
)
"%NODE%" "%~dp0predict_distance_10.20.js" %*
set "RESULT=%ERRORLEVEL%"
echo.
pause
exit /b %RESULT%
