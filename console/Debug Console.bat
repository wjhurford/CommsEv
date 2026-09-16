@echo off
REM Same as the normal launcher but keeps the window open so errors show.
setlocal
cd /d "%~dp0.."
echo Running CommsEv Console with output visible...
echo.
py "console\app.py"
echo.
echo --- app exited ---
pause
