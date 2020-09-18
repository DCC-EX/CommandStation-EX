ECHO ON
FOR /F "delims=" %%i IN ('dir %TMP%\arduino_build_* /b /ad-h /t:c /od') DO SET a=%%i
echo Most recent subfolder: %a% >%TMP%\OBJDUMP_%a%.txt
avr-objdump --private=mem-usage  %TMP%\%a%\DCCEX.ino.elf  >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %TMP%\%a%\DCCEX.ino.elf | find ".text" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %TMP%\%a%\DCCEX.ino.elf | find ".data" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %TMP%\%a%\DCC.ino.elf | find ".bss" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
notepad %TMP%\OBJDUMP_%a%.txt
EXIT
