@echo off
call conda activate weld
cd /d %~dp0
uvicorn 2:app --host 0.0.0.0 --port 8000
pause