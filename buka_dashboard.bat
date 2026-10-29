@echo off
title Dashboard Analisis Data Skripsi ESP32
echo ========================================================
echo Membuka Dashboard Analisis Data Skripsi di Browser...
echo ========================================================
cd /d "%~dp0\dashboard_web"
python server.py
pause
