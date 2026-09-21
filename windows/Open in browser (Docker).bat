@echo off
REM Builds the CommsEv image if needed, starts it, and opens the Console in
REM your browser. Needs Docker Desktop running. Nothing else - no Python.
cd /d "%~dp0.."
where docker >nul 2>nul || (echo Docker Desktop is not installed or not on PATH. & echo Get it from docker.com, start it, then run this again. & pause & exit /b 1)

REM Start Docker Desktop if its engine is not running yet (after a reboot,
REM typically), and wait for it - up to two minutes.
docker info >nul 2>nul && goto engine_up
echo Docker Desktop is not running - starting it...
if exist "%LOCALAPPDATA%\Programs\DockerDesktop\Docker Desktop.exe" start "" "%LOCALAPPDATA%\Programs\DockerDesktop\Docker Desktop.exe"
if exist "%ProgramFiles%\Docker\Docker\Docker Desktop.exe" start "" "%ProgramFiles%\Docker\Docker\Docker Desktop.exe"
set /a ENGINE_TRIES=0
:engine_wait
timeout /t 3 >nul
docker info >nul 2>nul && goto engine_up
set /a ENGINE_TRIES+=1
if %ENGINE_TRIES% GEQ 40 (echo. & echo *** Docker Desktop did not come up in two minutes. Open it from the Start menu, wait for "Engine running", then run this again. *** & pause & exit /b 1)
echo   waiting for the Docker engine...
goto engine_wait
:engine_up

REM Draw the Console at the size the browser will actually have in full
REM screen: the screen size AFTER Windows display scaling (a 1920x1080 laptop
REM at 125%% gives apps 1536x864). Anything else gets rescaled and looks soft.
for /f "tokens=1,2" %%w in ('powershell -NoProfile -Command "Add-Type -AssemblyName System.Windows.Forms; $b=[System.Windows.Forms.Screen]::PrimaryScreen.Bounds; Write-Output ($b.Width.ToString() + ' ' + $b.Height.ToString())"') do (set DISPLAY_WIDTH=%%w& set DISPLAY_HEIGHT=%%x)
REM Fall back to 1080p unless both values are plain numbers.
echo %DISPLAY_WIDTH%| findstr /r "^[0-9][0-9]*$" >nul || set DISPLAY_WIDTH=1920
echo %DISPLAY_HEIGHT%| findstr /r "^[0-9][0-9]*$" >nul || set DISPLAY_HEIGHT=1080
echo Browser full-screen size is %DISPLAY_WIDTH% x %DISPLAY_HEIGHT% - the Console will be drawn at exactly that.
echo.
echo Building and starting CommsEv (first time takes a few minutes)...
docker compose up -d --build
if errorlevel 1 (echo. & echo *** docker failed - copy the text above into a GitHub issue *** & pause & exit /b 1)
echo.
echo Waiting for the Console to come up...
set /a TRIES=0
:waitloop
powershell -NoProfile -Command "try { $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 2 http://localhost:5800; exit 0 } catch { exit 1 }" >nul 2>nul && goto up
set /a TRIES+=1
if %TRIES% GEQ 30 (echo. & echo *** The container started but nothing is answering on port 5800 after 60 s. *** & echo *** Double-click "windows\Docker log.bat" and send the output to Claude. *** & pause & exit /b 1)
timeout /t 2 >nul
goto waitloop
:up
echo Opening http://localhost:5800 ...
start "" http://localhost:5800
echo.
echo In the browser press F11 for full screen - it is drawn 1:1 at that size.
echo To stop it later:  docker compose down   (or stop it in Docker Desktop)
pause
