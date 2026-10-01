@echo off
setlocal
rem Usage: build_flash.bat [COM port or --build-only] [--no-pause]
set "PPM_FLASH_PORT=%~1"
set "PPM_BUILD_ONLY="
if /I "%~1"=="--build-only" set "PPM_BUILD_ONLY=1"
if not defined PPM_FLASH_PORT set /p "PPM_FLASH_PORT=Enter the Heltec serial port (e.g. COM7): "
set "PPM_IDF_PROFILE=C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1"
set "RESULT=1"

if not exist "%PPM_IDF_PROFILE%" (
    echo ERROR: ESP-IDF environment profile not found:
    echo %PPM_IDF_PROFILE%
    goto finish
)

rem Build from the firmware directory, even when launched from Explorer.
pushd "%~dp0"
if errorlevel 1 goto finish

rem Use the installed environment for C:\esp\v6.1\esp-idf.
rem idf.py stops before flashing if the build fails.
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference = 'Stop'; try { if (-not $env:PPM_BUILD_ONLY -and $env:PPM_FLASH_PORT -notmatch '^COM[0-9]+$') { throw 'Specify the Heltec serial port, such as COM7.' }; . $env:PPM_IDF_PROFILE; $env:PYTHONUTF8 = '1'; $env:IDF_CCACHE_ENABLE = '0'; $env:IDF_TARGET = 'esp32s3'; if ($env:PPM_BUILD_ONLY) { Write-Host 'Building ppm_tx (no flash)'; Invoke-idfpy build } else { Write-Host ('Building and flashing ppm_tx on ' + $env:PPM_FLASH_PORT); Invoke-idfpy -p $env:PPM_FLASH_PORT build flash }; exit $LASTEXITCODE } catch { Write-Error $_ -ErrorAction Continue; exit 1 }"
set "RESULT=%ERRORLEVEL%"
popd

:finish
echo.
if "%RESULT%"=="0" (
    echo Requested build/flash steps completed successfully.
) else (
    echo Build or flash failed. Review the error above.
)
rem Keep the window open after a double-click; allow unattended terminal use.
if /I not "%~2"=="--no-pause" pause
exit /b %RESULT%
