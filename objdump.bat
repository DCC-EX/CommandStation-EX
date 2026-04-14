ECHO ON
FOR /F "delims=" %%i IN ('dir %TMP%\CommandStation-EX.ino.elf /s /b /o-D') DO SET ELF=%%i
SET DUMP=%TEMP%\OBJDUMP.txt
echo Most recent subfolder: %ELF% >%DUMP%

set PATH="C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\";%PATH%
avr-objdump --private=mem-usage  %ELF%  >>%DUMP%
ECHO ++++++++++++++++++++++++++++++++++ >>%DUMP%
avr-objdump -x -C %ELF% | find ".text" | sort /+25 /R >>%DUMP%
ECHO ++++++++++++++++++++++++++++++++++ >>%DUMP%
avr-objdump -x -C %ELF% | find ".data" | sort /+25 /R >>%DUMP%
ECHO ++++++++++++++++++++++++++++++++++ >>%DUMP%
avr-objdump -x -C %ELF% | find ".bss" | sort /+25 /R >>%DUMP%
ECHO ++++++++++++++++++++++++++++++++++ >>%DUMP%
avr-objdump -D -S %ELF%  >>%DUMP%
%DUMP%
EXIT
