<#
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
#>

<############################################
For script errors set ExecutionPolicy:
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy Bypass
############################################>

<############################################
Optional command line parameters:
  $buildDirectory - specify an existing directory rather than generating a new unique one
  $version - specify an exact version to download
############################################>
Param(
  [Parameter()]
  [String]$buildDirectory,
  [Parameter()]
  [String]$version
)

<############################################
Define global parameters here such as known URLs etc.
############################################>
$installerVersion = "v0.0.1"
$gitHubAPITags = "https://api.github.com/repos/DCC-EX/CommandStation-EX/git/refs/tags"
$gitHubURLPrefix = "https://github.com/DCC-EX/CommandStation-EX/archive/"
if ((Get-WmiObject win32_operatingsystem | Select-Object osarchitecture).osarchitecture -eq "64-bit") {
  $arduinoCLIURL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip"
  $arduinoCLIZip = $env:TEMP + "\" + "arduino-cli_latest_Windows_64bit.zip"
} else {
  $arduinoCLIURL = "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_32bit.zip"
  $arduinoCLIZip = $env:TEMP + "\" + "arduino-cli_latest_Windows_32bit.zip"
}
$arduinoCLIDirectory = $env:TEMP + "\" + "arduino-cli_installer"
$arduinoCLI = $arduinoCLIDirectory + "\arduino-cli.exe"

<############################################
Set default action for progress indicators, warnings, and errors
############################################>
$global:ProgressPreference = "SilentlyContinue"
$global:WarningPreference = "SilentlyContinue"
$global:ErrorActionPreference = "SilentlyContinue"

<############################################
If $buildDirectory not provided, generate a new time/date stamp based directory to use
############################################>
if (!$PSBoundParameters.ContainsKey('buildDirectory')) {
  $buildDate = Get-Date -Format 'yyyyMMdd-HHmmss'
  $buildDirectory = $env:TEMP + "\" + $buildDate
}
$commandStationDirectory = $buildDirectory + "\CommandStation-EX"

<############################################
Write out intro message and prompt to continue
############################################>
@"
Welcome to the DCC-EX PowerShell installer for EX-CommandStation ($installerVersion)

Current installer options:
- EX-CommandStation will be built in $commandStationDirectory
- Arduino CLI will be in $arduinoCLIDirectory
"@


<############################################
Create build directory if it doesn't exist, or fail
############################################>
if (!(Test-Path -PathType Container -Path $buildDirectory)) {
  try {
    New-Item -ItemType Directory -Path $buildDirectory | Out-Null
  }
  catch {
    Write-Output "Could not create build directory $buildDirectory"
    Exit
  }
}

<############################################
See if we have the Arduino CLI already, otherwise download and extract it
############################################>
if (!(Test-Path -PathType Leaf -Path $arduinoCLI)) {
  if (!(Test-Path -PathType Container -Path $arduinoCLIDirectory)) {
    try {
      New-Item -ItemType Directory -Path $arduinoCLIDirectory | Out-Null
    }
    catch {
      Write-Output "Arduino CLI does not exist and cannot create directory $arduinoCLIDirectory"
      Exit
    }
  }
  Write-Output "Downloading and extracting Arduino CLI"
  try {
    Invoke-WebRequest -Uri $arduinoCLIURL -OutFile $arduinoCLIZip
  }
  catch {
    Write-Output "Failed to download Arduino CLI"
    Exit
  }
  try {
    Expand-Archive -Path $arduinoCLIZip -DestinationPath $arduinoCLIDirectory -Force
  }
  catch {
    Write-Output "Failed to extract Arduino CLI"
  }
}

<#
Write-Output "Installing using directory $buildDirectory"

$tagList = Invoke-RestMethod -Uri $gitHubAPITags

foreach ($tag in $tagList) {
  $version = $tag.ref.split("/")[2]
  $versionURL = $gitHubURLPrefix + $tag.ref
  Write-Output "$version : $versionURL"
}
#>