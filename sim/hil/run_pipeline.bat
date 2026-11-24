@echo off
setlocal
color 0B

echo ===================================================
echo   ESC DIGITAL TWIN - FULL PIPELINE
echo ===================================================
echo.

:: Check if Python is available
where python >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Python is not in your PATH.
    pause
    exit /b 1
)

:: Step 1
echo [1/5] Extracting netlist from KiCad PCB...
python extraction.py
if %ERRORLEVEL% neq 0 goto :error
echo.

:: Step 2
echo [2/5] Analyzing and classifying components...
python netlist_analyzer.py
if %ERRORLEVEL% neq 0 goto :error
echo.

:: Step 3
echo [3/5] Compiling SPICE netlist...
python spice_compiler.py
if %ERRORLEVEL% neq 0 goto :error
echo.

:: Step 4
echo [4/5] Running ngspice .op simulation (30A Load)...
python foc_spice_solver.py
if %ERRORLEVEL% neq 0 goto :error
echo.

:: Step 5
echo [5/5] Generating Thermal Map (LIVE)...
python thermal_viewer.py --heatsink
if %ERRORLEVEL% neq 0 goto :error
echo.

echo ===================================================
echo   [+] PIPELINE COMPLETED SUCCESSFULLY!
echo ===================================================
pause
exit /b 0

:error
echo.
echo ===================================================
echo   [!] PIPELINE FAILED AT THE STEP ABOVE.
echo ===================================================
pause
exit /b 1