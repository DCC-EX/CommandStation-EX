#!/bin/bash

# Find avr-objdump that matches the installed arduino binary
ARDUINOBIN=$(ls -l $(type -p arduino)| awk '{print $NF ; exit 0}')
PATH=$(dirname "$ARDUINOBIN")/hardware/tools/avr/bin:$PATH

LASTBUILD=$(ls -tr /tmp/arduino_build_*/*.ino.elf | tail -1)
avr-objdump  --private=mem-usage  "$LASTBUILD"

for segment in .text .data .bss ; do
    echo '++++++++++++++++++++++++++++++++++'
    avr-objdump  -x -C "$LASTBUILD" | awk '$2 == "'$segment'" && $3 != 0 {print $3,$2} ; $4 == "'$segment'" && $5 != 0 { print $5,$6}' | sort -r
done
