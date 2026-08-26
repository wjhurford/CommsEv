@echo off
REM Deadband Console launcher. Double-click this file.
REM Tries the windowed Python launcher first so no black terminal appears.
setlocal
cd /d "%~dp0.."

where pyw >nul 2>nul && (start "" pyw "console\app.py" & exit /b)
where pythonw >nul 2>nul && (start "" pythonw "console\app.py" & exit /b)
where py >nul 2>nul && (py "console\app.py" & exit /b)

echo Could not find Python.
echo Install it from python.org and tick "Add Python to PATH".
pause
