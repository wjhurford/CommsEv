@echo off
REM Runs the whole CommsEv test suite, keeps the window open, and saves the
REM full output to test_output.txt so a failure can be read back later.
REM Expect the last line to read: N passed, 0 failed
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
%PY% tests\test_all.py > test_output.txt 2>&1
findstr /C:"FAIL" /C:"passed" /C:"Error" test_output.txt
echo.
echo (full output saved to test_output.txt)
pause
