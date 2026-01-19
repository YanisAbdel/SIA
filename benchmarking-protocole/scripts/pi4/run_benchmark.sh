#!/bin/bash


if [ ! -f "data/best.onnx" ]; then
    echo "ERREUR : Fichier data/best.onnx manquant !"
    echo "Veuillez transférer best.onnx depuis votre PC."
    exit 1
fi

# Activation venv
if [ -f venv/bin/activate ]; then
    echo "Activation du venv..."
    source venv/bin/activate
fi

echo "========================================================"
echo "BENCHMARK RASPBERRY PI 4 (CPU + ONNX)"
echo "========================================================"

echo ""
echo "[ETAPE 1/3] Mesure PLANCHER..."
python3 scripts/benchmark_plancher.py

echo ""
echo "PAUSE (30s)..."
sleep 30

echo ""
echo "[ETAPE 2/3] Mesure CHARGE (Attention à la surchauffe)..."
python3 scripts/benchmark_charge.py

echo ""
echo "[ETAPE 3/3] Analyse..."
python3 scripts/analyze_results.py

echo ""
echo "TERMINÉ !"