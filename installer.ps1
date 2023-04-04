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
# 
# For script errors set ExecutionPolicy:
# Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy Bypass

Param(
  [Parameter()]
  [String]$buildDirectory
)

if (!$PSBoundParameters.ContainsKey('buildDirectory')) {
# Use the current date/time stamp to create a unique directory if one is not specified.
  $buildDate = Get-Date -Format 'yyyyMMdd-HHmmss'
  $buildDirectory = $env:TEMP + "\" + $buildDate
}

$gitHubAPITags = "https://api.github.com/repos/DCC-EX/CommandStation-EX/git/refs/tags"

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

$tagList = Invoke-RestMethod -Uri $gitHubAPITags

# Example zip: https://github.com/DCC-EX/CommandStation-EX/archive/refs/tags/v4.2.36-Devel.zip

foreach ($tag in $tagList) {
  $version = $tag.ref.split("/")[2]
  $versionURL = "https://github.com/DCC-EX/CommandStation-EX/archive/" + $tag.ref
  Write-Output "$version : $versionURL"
}
