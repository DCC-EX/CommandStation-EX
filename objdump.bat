ECHO ON
FOR /F "delims=" %%i IN ('dir %TMP%\arduino_build_* /b /ad-h /t:c /od') DO SET a=%%i
echo Most recent subfolder: %a% >%TMP%\OBJDUMP_%a%.txt
SET ELF=%TMP%\%a%\CommandStation-EX.ino.elf
set PATH="C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\";%PATH%
avr-objdump --private=mem-usage  %ELF%  >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %ELF% | find ".text" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %ELF% | find ".data" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -x -C %ELF% | find ".bss" | sort /+25 /R >>%TMP%\OBJDUMP_%a%.txt
ECHO ++++++++++++++++++++++++++++++++++ >>%TMP%\OBJDUMP_%a%.txt
avr-objdump -D -S %ELF%  >>%TMP%\OBJDUMP_%a%.txt
%TMP%\OBJDUMP_%a%.txt
EXIT
