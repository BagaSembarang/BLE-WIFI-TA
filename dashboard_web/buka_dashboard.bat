@echo off
title Dashboard Analisis Data Skripsi ESP32
echo ========================================================
echo Membuka Dashboard Analisis Data Skripsi di Browser...
echo ========================================================
cd /d "%~dp0"
python server.py
pause
