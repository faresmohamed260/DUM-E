@echo off
cd /d "%~dp0"
for /f "tokens=5" %%p in ('netstat -ano ^| findstr /R /C:":8876 .*LISTENING"') do (
  taskkill /PID %%p /F >nul 2>&1
)
start "" http://127.0.0.1:8876
python app.py

