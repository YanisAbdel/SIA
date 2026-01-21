#!/bin/bash

# Activation de l'environnement virtuel
if [ -f venv/bin/activate ]; then
    echo "Activation du venv..."
    source venv/bin/activate
else
    echo "ATTENTION : venv introuvable. Installation globale utilisée."
fi

echo "========================================================"
echo "BENCHMARK JETSON ORIN NANO (GPU + JTOP)"
echo "========================================================"

# ETAPE 1 : MESURE AU REPOS
echo ""
echo "[ETAPE 1/3] Mesure PLANCHER (Consommation au repos)..."
python3 scripts/benchmark_plancher.py

# PAUSE THERMIQUE
echo ""
echo "PAUSE REFROIDISSEMENT (60s)..."
echo "Laissez la carte redescendre en température."
sleep 60

# ETAPE 2 : MESURE EN CHARGE
echo ""
echo "[ETAPE 2/3] Mesure CHARGE (Inférence YOLO GPU)..."
python3 scripts/benchmark_charge.py

# ETAPE 3 : ANALYSE
echo ""
echo "[ETAPE 3/3] Génération du Rapport..."
python3 scripts/analyze_results.py

echo ""
echo "TERMINÉ ! Vérifiez le dossier results/"