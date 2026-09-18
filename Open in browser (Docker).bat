@echo off
REM Builds the CommsEv image if needed, starts it, and opens the Console in
REM your browser. Needs Docker Desktop running. Nothing else - no Python.
cd /d "%~dp0"
where docker >nul 2>nul || (echo Docker Desktop is not installed or not on PATH. & echo Get it from docker.com, start it, then run this again. & pause & exit /b 1)

REM Draw the Console at the size the browser will actually have in full
REM screen: the screen size AFTER Windows display scaling (a 1920x1080 laptop
REM at 125%% gives apps 1536x864). Anything else gets rescaled and looks soft.
for /f "tokens=1,2" %%w in ('powershell -NoProfile -Command "Add-Type -AssemblyName System.Windows.Forms; $b=[System.Windows.Forms.Screen]::PrimaryScreen.Bounds; Write-Output ($b.Width.ToString() + ' ' + $b.Height.ToString())"') do (set DISPLAY_WIDTH=%%w& set DISPLAY_HEIGHT=%%h)
if "%DISPLAY_WIDTH%"=="" set DISPLAY_WIDTH=1920
if "%DISPLAY_HEIGHT%"=="" set DISPLAY_HEIGHT=1080
echo Browser full-screen size is %DISPLAY_WIDTH% x %DISPLAY_HEIGHT% - the Console will be drawn at exactly that.
echo.
echo Building and starting CommsEv (first time takes a few minutes)...
docker compose up -d --build
if errorlevel 1 (echo. & echo *** docker failed - copy the text above into a GitHub issue *** & pause & exit /b 1)
echo.
echo Opening http://localhost:5800 ...
timeout /t 5 >nul
start "" http://localhost:5800
echo.
echo In the browser press F11 for full screen - it is drawn 1:1 at that size.
echo To stop it later:  docker compose down   (or stop it in Docker Desktop)
pause
