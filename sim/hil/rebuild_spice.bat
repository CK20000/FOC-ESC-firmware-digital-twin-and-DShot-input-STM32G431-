@echo off
title Digital Twin - Rebuild SPICE (FOC ESC)
cd /d "%~dp0"

echo [1/4] Exporting schematic netlist...
python export_schematic_netlist.py
if errorlevel 1 goto :fail

echo.
echo [2/4] Classifying components...
python netlist_analyzer.py
if errorlevel 1 goto :fail

echo.
echo [3/4] Compiling SPICE netlist...
python spice_compiler.py
if errorlevel 1 goto :fail

echo.
echo [4/4] Running DC scenarios...
python dc_scenario_solver.py
if errorlevel 1 goto :fail

echo.
echo ================================================
echo   SPICE rebuild complete.
echo ================================================
pause
exit

:fail
echo.
echo [!] Step failed. See error above.
pause
exit /b 1