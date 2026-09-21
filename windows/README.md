# windows/

Double-click launchers for Windows. Each changes to the repository root
before running, so they work from this folder.

| File | Purpose |
| --- | --- |
| `Setup (run once).bat` | Installs `requirements.txt` into the Windows Python. |
| `CommsEv Console.bat` | Starts the Console natively (no terminal window). |
| `Debug Console.bat` | Starts the Console with the terminal kept open, so a crash is readable. |
| `Run tests.bat` | Runs `tests/test_all.py`; saves the full output to `test_output.txt`. |
| `Open in browser (Docker).bat` | Starts Docker Desktop if needed, builds and starts the container at the screen's own resolution, opens <http://localhost:5800>. |
| `Docker log.bat` | Shows the container's state and last 80 log lines. |

Linux and macOS use the commands in the top-level `README.md` directly.
