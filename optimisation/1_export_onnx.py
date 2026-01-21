from ultralytics import YOLO

# 1. Charge ton modèle entrainé
model = YOLO("best.pt")

# 2. Export vers ONNX
# imgsz : La taille d'image utilisée pendant l'entraînement (640 par défaut)
# opset : Version du standard ONNX (12 est très stable pour OpenVINO)
print("Export en cours vers ONNX...")
success = model.export(format="onnx", imgsz=640, opset=12)

if success:
    print("Export ONNX réussi ! 'best.onnx' est créé.")
else:
    print("Échec de l'export.")