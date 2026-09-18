@echo off
REM Builds the CommsEv image if needed, starts it, and opens the Console in
REM your browser. Needs Docker Desktop running. Nothing else - no Python.
cd /d "%~dp0"
where docker >nul 2>nul || (echo Docker Desktop is not installed or not on PATH. & echo Get it from docker.com, start it, then run this again. & pause & exit /b 1)
echo Building and starting CommsEv (first time takes a few minutes)...
docker compose up -d --build
if errorlevel 1 (echo. & echo *** docker failed - copy the text above into a GitHub issue *** & pause & exit /b 1)
echo.
echo Opening http://localhost:5800 ...
timeout /t 5 >nul
start "" http://localhost:5800
echo.
echo To stop it later:  docker compose down   (or stop it in Docker Desktop)
pause
