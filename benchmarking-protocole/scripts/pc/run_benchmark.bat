@echo off
title BENCHMARK PC NVIDIA
color 0A

echo ========================================================
echo  BENCHMARK PC (NVIDIA-SMI)
echo ========================================================

:: Activation venv
if exist venv\Scripts\activate.bat (
    echo [INFO] Activation du venv...
    call venv\Scripts\activate.bat
) else (
    echo [WARN] Pas de venv trouve, on tente avec le Python global...
)

echo.
echo [ETAPE 1/3] Mesure PLANCHER (Idle GPU)...
python scripts\benchmark_plancher.py

echo.
echo [PAUSE] Refroidissement (30s)...
timeout /t 30 /nobreak >nul

echo.
echo [ETAPE 2/3] Mesure CHARGE (YOLO CUDA)...
python scripts\benchmark_charge.py

echo.
echo [ETAPE 3/3] Analyse et Rapport...
python scripts\analyze_results.py

echo.
echo ========================================================
echo  TEST TERMINE - Résultats dans results/pc/
echo ========================================================
pause