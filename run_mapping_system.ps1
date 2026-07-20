# Ping360 3D Mapping Engine Startup Utility

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "      PING360 3D SONAR MAPPING ENGINE RUNNER              " -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "Choose the execution mode:" -ForegroundColor White
Write-Host "  1. Emulator / Sandbox Mode (Simulated DVL, IMU, and Sonar)" -ForegroundColor Green
Write-Host "  2. Real Hardware Mode (Connects to actual serial/UDP sensors)" -ForegroundColor Yellow
Write-Host ""

$choice = Read-Host "Select option [1-2] (Default is 2)"
if ($choice -eq "") { $choice = "2" }

if ($choice -eq "1") {
    Write-Host "`nStarting in EMULATOR mode..." -ForegroundColor Green
    Write-Host "HTTP Dashboard: http://localhost:8000" -ForegroundColor Cyan
    Write-Host "WebSocket link: ws://localhost:8001" -ForegroundColor Cyan
    Write-Host "Press Ctrl+C in this terminal window to stop the servers.`n" -ForegroundColor DarkGray
    
    .\venv\Scripts\python.exe mapper_3d_engine.py --emulate
}
elseif ($choice -eq "2") {
    $port = Read-Host "`nEnter Ping360 Serial Port or IP (Default: 192.168.2.2:9092)"
    if ($port -eq "") { $port = "192.168.2.2:9092" }
    
    Write-Host "`nStarting in REAL HARDWARE mode (Ping360: $port)..." -ForegroundColor Yellow
    Write-Host "Attitude Telemetry: UDP Port 14550 (IP: 192.168.2.2)" -ForegroundColor Cyan
    Write-Host "DVL TCP JSON stream: Port 16171 (IP: 192.168.2.3)" -ForegroundColor Cyan
    Write-Host "HTTP Dashboard: http://localhost:8000" -ForegroundColor Cyan
    Write-Host "Press Ctrl+C in this terminal window to stop the servers.`n" -ForegroundColor DarkGray
    
    .\venv\Scripts\python.exe mapper_3d_engine.py --connection $port
}
else {
    Write-Host "Invalid option. Exiting." -ForegroundColor Red
}
