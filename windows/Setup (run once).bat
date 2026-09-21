@echo off
REM Installs everything the Console needs into WINDOWS Python.
REM Run this from Windows, not WSL.
cd /d "%~dp0.."
echo Installing CommsEv Console dependencies...
echo.
py -m pip install -r requirements.txt
echo.
if %errorlevel%==0 (echo Done. You can now run "windows\CommsEv Console.bat".) else (echo Something went wrong - copy the text above into a GitHub issue.)
pause
