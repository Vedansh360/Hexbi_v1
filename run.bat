@echo off
echo Activating conda environment: hexbi_v1_conn1
call conda activate hexbi_v1_conn1

echo Changing directory to E:/Codes/Hexbi_v1
cd /d E:\Codes\Hexbi_v1

echo Starting WebSocket server...
start cmd /k "python websocket_server_camera.py"

timeout /t 5 >nul  REM Wait for the server to start

echo Starting WebSocket client...
start cmd /k "python ws_client.py"

echo All processes started. Press any key to exit...
pause
