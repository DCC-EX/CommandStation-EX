@ECHO OFF

FOR /f "tokens=*" %%a IN ('powershell Get-ExecutionPolicy -Scope CurrentUser') DO SET PS_POLICY=%%a

IF NOT %PS_POLICY=="Bypass" (
  powershell Set-ExecutionPolicy -Scope CurrentUser Bypass
)

powershell %~dp0%installer.ps1

IF NOT %PS_POLICY=="Bypass" (
  powershell Set-ExecutionPolicy -Scope CurrentUser %PS_POLICY%
)
