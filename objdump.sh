#!/bin/bash

# Find avr-objdump that matches the installed arduino binary
ARDUINOBIN=$(ls -l $(type -p arduino)| awk '{print $NF ; exit 0}')
PATH=$(dirname "$ARDUINOBIN")/hardware/tools/avr/bin:$PATH

avr-objdump  --private=mem-usage  /tmp/arduino_build_233823/Blinkhabaplus.ino.elf

for segment in .text .data .bss ; do
    echo '++++++++++++++++++++++++++++++++++'
    avr-objdump  -x -C /tmp/arduino_build_233823/Blinkhabaplus.ino.elf  | awk '$2 == "'$segment'" && $3 != 0 {print $3,$2} ; $4 == "'$segment'" && $5 != 0 { print $5,$6}' | sort -r
done
