@echo off
setlocal
rem Usage: build_flash.bat [COM port] [--no-pause]
set "XIAO_FLASH_PORT=%~1"
if not defined XIAO_FLASH_PORT set "XIAO_FLASH_PORT=COM8"
set "XIAO_IDF_PROFILE=C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1"
set "RESULT=1"

if not exist "%XIAO_IDF_PROFILE%" (
    echo ERROR: ESP-IDF environment profile not found:
    echo %XIAO_IDF_PROFILE%
    goto finish
)

rem Build from the firmware directory, even when launched from Explorer.
pushd "%~dp0"
if errorlevel 1 goto finish

rem Use the installed environment for C:\esp\v6.1\esp-idf.
rem idf.py stops before flashing if the build fails.
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference = 'Stop'; try { if ($env:XIAO_FLASH_PORT -notmatch '^COM[0-9]+$') { throw 'Specify a serial port such as COM8.' }; . $env:XIAO_IDF_PROFILE; $env:PYTHONUTF8 = '1'; $env:IDF_CCACHE_ENABLE = '0'; Write-Host ('Building and flashing XIAO on ' + $env:XIAO_FLASH_PORT); Invoke-idfpy -p $env:XIAO_FLASH_PORT build flash; exit $LASTEXITCODE } catch { Write-Error $_ -ErrorAction Continue; exit 1 }"
set "RESULT=%ERRORLEVEL%"
popd

:finish
echo.
if "%RESULT%"=="0" (
    echo Build and flash completed successfully.
) else (
    echo Build or flash failed. Review the error above.
)
rem Keep the window open after a double-click; allow unattended terminal use.
if /I not "%~2"=="--no-pause" pause
exit /b %RESULT%
