@echo off
echo Activation de l'environnement...
call .\venv\Scripts\activate

echo 1. Lancement de l'export ONNX...
python 1_export_onnx.py

echo 2. Conversion en BLOB pour OAK-D...
python -m blobconverter -onnx best.onnx -sh 6 -o . --no-cache

echo Fichier .blob cree
pause