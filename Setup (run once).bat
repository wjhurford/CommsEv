@echo off
REM Installs everything the Console needs into WINDOWS Python.
REM Run this from Windows, not WSL.
cd /d "%~dp0"
echo Installing Deadband Console dependencies...
echo.
py -m pip install -r requirements-console.txt
echo.
if %errorlevel%==0 (echo Done. You can now run "console\Deadband Console.bat".) else (echo Something went wrong - copy the text above and send it to Claude.)
pause
