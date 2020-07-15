ECHO ON
FOR /F "delims=" %%i IN ('dir %TMP%\arduino_build_* /b /ad-h /t:c /od') DO SET a=%%i
echo Most recent subfolder: %a%
avr-objdump -x -C %TMP%\%a%\CVReader.ino.elf | find ".text" | sort /+25 /R >%TMP%\OBJDUMP_%a%.txt
notepad %TMP%\OBJDUMP_%a%.txt

