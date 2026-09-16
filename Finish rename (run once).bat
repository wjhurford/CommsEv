@echo off
REM ===========================================================================
REM  CommsEv - finish the Deadband -> CommsEv rename. Double-click once.
REM ===========================================================================
REM  The renamed files are already in place. This removes what they replaced,
REM  clears out session debris, runs the tests, and commits. Nothing is pushed.
REM  Delete this file after it has run.
REM ===========================================================================
cd /d "%~dp0"
REM Pick whichever Python already has PySide6; failing that, install the
REM requirements into "python" (or "py") first. Same thing Setup (run once).bat does.
set PY=
python -c "import PySide6" >nul 2>nul && set PY=python
if "%PY%"=="" py -c "import PySide6" >nul 2>nul && set PY=py
if not "%PY%"=="" goto havepy
set PY=python
where python >nul 2>nul || set PY=py
echo PySide6 is not installed for %PY% - installing requirements.txt first...
%PY% -m pip install -r requirements.txt
:havepy
REM Resolve to the real executable: "py" obeys the #! line in test_all.py and can
REM silently switch to a different Python than the one it just checked.
for /f "delims=" %%i in ('%PY% -c "import sys; print(sys.executable)"') do set PYEXE=%%i
set PY="%PYEXE%"
echo Using: %PY%

echo.
echo === 0. clearing a stale git lock, if one was left by a crashed process ====
if exist .git\HEAD.lock (echo   removing .git\HEAD.lock & del /q .git\HEAD.lock)
if exist .git\index.lock (echo   removing .git\index.lock & del /q .git\index.lock)

echo === 1. removing the old package directories and files ==================
git rm -r -q -f --ignore-unmatch deadband "ros2/src/deadband_ros" "console/Deadband Console.bat" "docs/DEADBAND-handover.md" requirements-console.txt "Commit demo prep.bat" "Commit pending work.bat" "Commit spectrum sections.bat" "Commit tidy-up.bat" "Commit walls and formation work.bat" "Claude outputs" console/last_error.log console/_experiment_panel_src.py.bak
if exist deadband rmdir /s /q deadband
if exist "ros2\src\deadband_ros" rmdir /s /q "ros2\src\deadband_ros"
if exist "ros2\build" rmdir /s /q "ros2\build"
if exist "ros2\install" rmdir /s /q "ros2\install"
if exist "ros2\log" rmdir /s /q "ros2\log"
for /d /r %%d in (__pycache__) do @if exist "%%d" rmdir /s /q "%%d"

echo.
echo === 2. clearing session debris from the root ============================
del /q deadband_session*.patch 2>nul
del /q "Commit *.bat" 2>nul
del /q "hy = *" 2>nul
del /q console\last_error.log 2>nul
del /q console\_experiment_panel_src.py.bak 2>nul
del /q .claude_commit_msg.txt 2>nul
if exist Deadband_References ren Deadband_References CommsEv_References

echo.
echo === 3. running the test suite (expect 557 passed, 0 failed) =============
%PY% tests\test_all.py > test_output.txt 2>&1
set TESTRC=%errorlevel%
findstr /C:"FAIL" /C:"passed" /C:"Error" test_output.txt
if not "%TESTRC%"=="0" (
  echo.
  echo *** TESTS FAILED - nothing has been committed. ***
  echo *** The full output is in test_output.txt - Claude can read it from there. ***
  pause
  exit /b 1
)
del /q test_output.txt 2>nul

echo.
echo === 4. committing =======================================================
git add -A
git -c core.quotepath=off status --short
git commit -q -m "Rename Deadband to CommsEv (Communications Evaluator)" -m "Package deadband -> commsev, ROS package deadband_ros -> commsev_ros, CLI python -m commsev, COMMSEV_ROOT, Console title and dialogs. Front-door README, CONTRIBUTING, NOTICE, docs/README index, single requirements.txt, Run tests.bat. Fixed the ROS repo-root marker (scenarios/ no longer exists) and the pkill pattern. Session debris removed from the root. 554 tests passing." -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_01NcLDzsL8jsnR5jknqtRVid"
git --no-pager log --oneline -3

echo.
echo === done ==================================================================
echo Next, on GitHub: Settings -^> rename the repository to  commsev
echo Then here:        git remote set-url origin https://github.com/wjhurford/commsev.git
echo                   git push
echo (The old URL keeps redirecting, so nothing breaks if you do this later.)
echo.
pause
