@echo off
setlocal
pushd "%~dp0" || exit /b 1

set "TESTER_PYTHON=.venv-ble\Scripts\python.exe"
if not exist "%TESTER_PYTHON%" (
    where py >nul 2>&1
    if errorlevel 1 (
        python -m venv .venv-ble
    ) else (
        py -3 -m venv .venv-ble
    )
    if errorlevel 1 goto failed
)

"%TESTER_PYTHON%" -c "import bleak" >nul 2>&1
if errorlevel 1 (
    "%TESTER_PYTHON%" -m pip install "bleak>=1.1,<4"
    if errorlevel 1 goto failed
)

"%TESTER_PYTHON%" -m tools.ble_tester
if errorlevel 1 goto failed
popd
exit /b 0

:failed
echo.
echo Could not start the BLE tester. Check the error above.
pause
popd
exit /b 1
