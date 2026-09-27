# Builds the ROS-free estimator core + offline replay tool with zig c++ (no ROS needed).
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$pkg = Join-Path $root "ros2_ws\src\tram_odometry"
New-Item -ItemType Directory -Force (Join-Path $root "tools\_bin") | Out-Null
$src = @("json_lite", "geo", "route_map", "traction_model", "estimator") | ForEach-Object { Join-Path $pkg "src\core\$_.cpp" }
python -m ziglang c++ -std=c++17 -O2 -Wall -Wextra -Wno-nullability-completeness -Wno-unknown-warning-option `
  -I (Join-Path $pkg "include") @src (Join-Path $pkg "src\replay_main.cpp") -o (Join-Path $root "tools\_bin\replay.exe")
if ($LASTEXITCODE -ne 0) { throw "build failed" }
Write-Host "built tools\_bin\replay.exe"
