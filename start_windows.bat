@echo off
rem ODrive Actuator Bench - start on Windows (double-click this file).
rem The first run creates a private Python environment in ".venv" and installs what the app needs
rem (needs internet, takes a few minutes). After that it starts right away.
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto install

set "PY="
where py >nul 2>nul
if not errorlevel 1 (
    for %%v in (3.13 3.12 3.11 3.10 3.14) do (
        if not defined PY (
            py -%%v -c "import sys" >nul 2>nul && set "PY=py -%%v"
        )
    )
)
if not defined PY (
    python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul && set "PY=python"
)
if not defined PY (
    echo.
    echo Python 3.10 or newer is needed and was not found.
    echo Download it from https://www.python.org/downloads/ and tick "Add python.exe to PATH" during setup,
    echo then run this file again.
    start "" "https://www.python.org/downloads/windows/"
    pause
    exit /b 1
)
echo Creating the app environment in .venv ...
%PY% -m venv .venv
if errorlevel 1 goto fail

:install
fc /b requirements.txt .venv\installed-requirements.txt >nul 2>nul
if not errorlevel 1 goto run
echo Installing the required packages - first run only, a few minutes ...
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 goto fail
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto fail
".venv\Scripts\python.exe" -m pip install odrive
if errorlevel 1 echo Note: the optional official odrive library was not installed. The USB serial connection still works.
copy /y requirements.txt .venv\installed-requirements.txt >nul

:run
".venv\Scripts\python.exe" -m actuator_bench %*
exit /b %errorlevel%

:fail
echo.
echo Setup failed. Check the internet connection and run this file again.
pause
exit /b 1
