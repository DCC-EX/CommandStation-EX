@echo off
REM ESP32 Backtrace Decoder Script
REM Usage: decode_bt.bat 0xADDR1 0xADDR2 ...

setlocal

set ELF=.pio\build\ESP32\firmware.elf
set ADDR2LINE="%USERPROFILE%\.platformio\packages\toolchain-xtensa-esp32\bin\xtensa-esp32-elf-addr2line.exe"

if not exist %ELF% (
  echo Firmware ELF not found at %ELF%
  exit /b 1
)

if "%1"=="" (
  echo Usage: %0 0xADDR1 0xADDR2 ...
  exit /b 1
)

echo Decoding addresses using %ELF%

%ADDR2LINE% -pfiaC -e %ELF% %*
