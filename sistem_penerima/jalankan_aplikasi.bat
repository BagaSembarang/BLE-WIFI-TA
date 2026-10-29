@echo off
title Sistem Penerima Game Controller ESP32
echo ========================================================
echo Membuka Aplikasi Sistem Penerima Laptop (GUI)...
echo ========================================================
cd /d "%~dp0"
python app_penerima_gui.py
pause
