#
# © 2023 Peter Cole
# 
# This file is part of EX-CommandStation
#
# This is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# It is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
#

# 32/64 bit Win installer
# https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_32bit.zip
# https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip

param(
  [Parameter()]
  [String]$BUILD_DIR
)

if (!$PSBoundParameters.ContainsKey('BUILD_DIR')) {
# Use the current date/time stamp to create a unique directory if one is not specified.
  $BUILD_DATE = Get-Date -Format 'yyyyMMdd-HHmmss'
  $BUILD_DIR = $env:TEMP + "\" + $BUILD_DATE
}

if ((Get-WmiObject win32_operatingsystem | Select-Object osarchitecture).osarchitecture -eq "64-bit") {
  $URL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
  $OUTFILE = $env:TEMP + "\" + "arduino-cli_latest_Windows_64bit.zip"
} else {
  $URL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_32bit.zip"
  $OUTFILE = $env:TEMP + "\" + "arduino-cli_latest_Windows_32bit.zip"
}

Write-Output "Downloading installer to $OUTFILE"

$ProgressPreference = "SilentlyContinue"
Invoke-WebRequest -Uri $URL -OutFile $OUTFILE

$CLI_INSTALL = $env:TEMP + "\" + "arduino-cli_installer"

Expand-Archive -Path $OUTFILE -DestinationPath $CLI_INSTALL -Force
$ProgressPreference = "Continue"

Write-Output "Installing using directory $BUILD_DIR"