@echo off
title MoSim - Virtual Musculoskeletal Simulator
cls
echo ======================================================================
echo    __  __        _____ _           
echo   ^|  \/  ^|      / ____(_)          
echo   ^| \  / ^| ___ ^| (___  _ _ __ ___  
echo   ^| ^|\/^| ^|/ _ \ \___ \^| ^| '_ ` _ \ 
echo   ^| ^|  ^| ^| (_) ^|____) ^| ^| ^| ^| ^| ^| ^|
echo   ^|_^|  ^|_^|\___/^|_____/^|_^|_^| ^|_^| ^|_^|
echo.
echo   Next-Generation Virtual Musculoskeletal Simulator & Biomechanics Engine
echo   Repository: https://github.com/rawansayed/MoSim
echo ======================================================================
echo.
echo [1/3] Initializing MoSim WebGL Biomechanics Environment...
cd /d "%~dp0"

echo [2/3] Starting Local Simulation Server at http://127.0.0.1:8090 ...
start /b python -m http.server 8090 > nul 2>&1

echo [3/3] Launching MoSim in Default Browser...
start "" "http://127.0.0.1:8090/index.html"

echo.
echo ======================================================================
echo  MoSim is now RUNNING!
echo  URL: http://127.0.0.1:8090/index.html
echo.
echo  Features:
echo   * 3D Biomechanics Viewport (PBR Three.js with Muscle Bulking)
echo   * 3-Element Hill Musculotendon Active/Passive Force Tracking
echo   * Multi-Model Support: Lower Extremity & Upper Limb
echo   * OpenSim 4.x Interoperability (.osim XML and .mot Motion Exporter)
echo   * Forward Dynamics Runge-Kutta 4 Physics Engine
echo ======================================================================
echo Press any key to terminate the server and exit...
pause > nul
taskkill /f /im python.exe /fi "WINDOWTITLE eq MoSim*" > nul 2>&1
