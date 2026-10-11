@echo off
REM One command to run the robot dog: finds the robot, starts Qwen, opens the dashboard.
REM   start_robot.bat                 robot on its own "robotdog" Wi-Fi (192.168.4.1)
REM   start_robot.bat 172.20.10.5     robot at another address (shown on the Serial Monitor)
cd /d "%~dp0"
set IP=%1
if "%IP%"=="" set IP=192.168.4.1

echo === 1/4 Stuck-leg model
if not exist models\stuck_rf.joblib (
    echo No model yet: training one ^(about a minute^)...
    py logger.py --source fake --auto --trials 60 --duration 60 --fast --no-enc --out data\noenc || goto :fail
    py train.py data\noenc || goto :fail
) else (
    echo found models\stuck_rf.joblib
)

echo === 2/4 Qwen ^(Ollama^)
curl -s -o nul http://localhost:11434/api/tags
if errorlevel 1 (
    echo Ollama isn't running: starting it...
    start "Ollama" /min ollama serve
    timeout /t 5 /nobreak >nul
    curl -s -o nul http://localhost:11434/api/tags
    if errorlevel 1 echo !! Ollama still not answering. The dashboard will open with buttons only.
) else (
    echo Ollama is running
)

echo === 3/4 Robot at %IP%
py robot_link.py --robot-ip %IP% || goto :norobot

echo === 4/4 Dashboard
py dashboard.py --robot-ip %IP%
goto :eof

:norobot
echo.
echo !! Can't reach the robot at %IP%.
echo    - Is this laptop on the robot's Wi-Fi ^("robotdog", password robotdog123^) or the same hotspot?
echo    - Check the IP on the Serial Monitor, then run: start_robot.bat THAT_IP
pause
exit /b 1

:fail
echo !! Something failed above.
pause
exit /b 1
