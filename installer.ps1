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

$gitHubAPITags = "https://api.github.com/repos/DCC-EX/CommandStation-EX/git/refs/tags"

param(
  [Parameter()]
  [String]$buildDirectory
)

if (!$PSBoundParameters.ContainsKey('buildDirectory')) {
# Use the current date/time stamp to create a unique directory if one is not specified.
  $buildDate = Get-Date -Format 'yyyyMMdd-HHmmss'
  $buildDirectory = $env:TEMP + "\" + $buildDate
}

if ((Get-WmiObject win32_operatingsystem | Select-Object osarchitecture).osarchitecture -eq "64-bit") {
  $arduinoCLIURL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
  $arduinoCLIZip = $env:TEMP + "\" + "arduino-cli_latest_Windows_64bit.zip"
} else {
  $arduinoCLIURL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_32bit.zip"
  $arduinoCLIZip = $env:TEMP + "\" + "arduino-cli_latest_Windows_32bit.zip"
}

Write-Output "Downloading installer to $arduinoCLIZip"

$ProgressPreference = "SilentlyContinue"
Invoke-WebRequest -Uri $arduinoCLIURL -OutFile $arduinoCLIZip

$arduinoCLIDirectory = $env:TEMP + "\" + "arduino-cli_installer"

Expand-Archive -Path $arduinoCLIZip -DestinationPath $arduinoCLIDirectory -Force
$ProgressPreference = "Continue"

Write-Output "Installing using directory $buildDirectory"

foreach ($tag in Invoke-RestMethod -Uri $gitHubAPITags | Format-List -Property ref) {
  $tag.getType()
}
