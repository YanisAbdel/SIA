import time
import cv2
import psutil
import pandas as pd
import onnxruntime as ort
import numpy as np
import os

# --- CONFIGURATION ---
MODEL_PATH = "data/best.onnx"
VIDEO_PATH = "data/benchmark_sequence.mp4"
OUTPUT_CSV = "results/charge.csv"
IMG_SIZE = 640

def get_cpu_temp():
    """Lit la température du CPU Raspberry Pi via le fichier système"""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read()) / 1000
    except:
        return 0.0

def preprocess(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose((2, 0, 1)) # HWC to CHW
    img = np.expand_dims(img, axis=0)
    img = img.astype(np.float32) / 255.0
    return img

def main():
    print(f"🚀 Démarrage Benchmark CHARGE (ONNX) sur {MODEL_PATH}...")
    
    # 1. Chargement Moteur
    try:
        session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
        input_name = session.get_inputs()[0].name
    except Exception as e:
        print(f"❌ Erreur chargement modèle : {e}")
        return

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("❌ Erreur: Impossible d'ouvrir la vidéo.")
        return

    data_log = []
    frame_count = 0
    start_time = time.time()

    print("▶️ TEST EN COURS... Regarde ton Wattmètre maintenant !")
    
    while True:
        ret, frame = cap.read()
        if not ret: break

        loop_start = time.time()

        # --- INFERENCE ---
        input_tensor = preprocess(frame)
        outputs = session.run(None, {input_name: input_tensor})
        # -----------------

        infer_end = time.time()
        
        # Calculs métriques
        latency = (infer_end - loop_start) * 1000 # ms
        fps = 1.0 / (infer_end - loop_start) if (infer_end - loop_start) > 0 else 0
        temp = get_cpu_temp()
        
        # Enregistrement avec les NOMS DE COLONNES EXACTS pour analyze_results.py
        data_log.append({
            "Phase": "TEST",            # Pour le filtre
            "FPS_Inst": round(fps, 1),
            "Latence_ms": round(latency, 2),
            "Temp_C": round(temp, 1),
            "Power_Soft_W": 0,          # Toujours 0 sur Pi
            "Power_Manual_W": 0         # On remplira ça après
        })

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"Frame {frame_count} | FPS: {fps:.1f} | Temp: {temp}°C")

    cap.release()
    
    # --- SAISIE MANUELLE DE LA PUISSANCE ---
    print("\n" + "="*40)
    print("🛑 FIN DU TEST VIDÉO")
    print("="*40)
    try:
        watts = float(input("👉 Entre la valeur moyenne lue sur le WATTMÈTRE (en W) : "))
    except ValueError:
        watts = 0.0
        print("⚠️ Erreur de saisie, on met 0W.")

    # Mise à jour des données avec la valeur saisie
    for row in data_log:
        row["Power_Manual_W"] = watts

    # Sauvegarde
    if not os.path.exists('results'): os.makedirs('results')
    df = pd.DataFrame(data_log)
    
    # On force l'ordre des colonnes pour faire propre
    cols = ["Phase", "FPS_Inst", "Latence_ms", "Temp_C", "Power_Soft_W", "Power_Manual_W"]
    df = df[cols]
    
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"\n✅ Terminé ! Résultats sauvegardés dans {OUTPUT_CSV}")
    print(f"   (Avec puissance manuelle fixée à {watts} W)")

if __name__ == "__main__":
    main()
