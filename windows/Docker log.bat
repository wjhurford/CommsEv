@echo off
REM Shows what the CommsEv container is doing / why it stopped. Copy the text to Claude.
cd /d "%~dp0.."
echo === container state ===
docker compose ps
echo.
echo === last 80 log lines ===
docker compose logs --tail 80 --no-color
echo.
pause
