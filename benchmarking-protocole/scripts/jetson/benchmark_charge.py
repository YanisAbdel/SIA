import cv2
import time
import csv
import psutil
import os
from datetime import datetime
from ultralytics import YOLO
from jtop import jtop

# --- CONFIGURATION ---
MODEL_PATH = "data/best.pt"
VIDEO_PATH = "data/benchmark_sequence.mp4"
OUTPUT_FILE = "results/charge.csv"
WARMUP_SEC = 10     
TEST_DURATION = 60 
IMG_SIZE = 640

def get_temp(jetson):
    t = jetson.stats.get('Temp GPU')
    if t is not None and t > 0: return t
    t = jetson.stats.get('GPU')
    if t is not None and t > 0: return t
    try: return jetson.temperature['GPU']['temp']
    except: return 0.0

def run_benchmark():
    print(f"--- MODE CHARGE GPU (JETSON) ---")
    
    if not os.path.exists('results'): os.makedirs('results')

    print("⏳ Chargement du modèle (Optimisation TensorRT possible)...")
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"❌ Erreur vidéo : {VIDEO_PATH}")
        return
    
    # Structure exacte pour analyze_results.py
    headers = ["Timestamp", "Frame_ID", "Latence_ms", "FPS_Inst", "CPU_%", "RAM_%", "Temp_C", "Power_Soft_W", "Phase", "Power_Manual_W"]
    
    print("🔌 Connexion aux sondes jtop...")
    with jtop() as jetson:
        if not jetson.ok(): return

        with open(OUTPUT_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
            
            start_global = time.time()
            frame_count = 0
            
            print(f"▶️ Démarrage du test ({TEST_DURATION}s)...")
            
            while True:
                elapsed = time.time() - start_global
                success, frame = cap.read()
                if not success: break 

                # Gestion des phases
                if elapsed < WARMUP_SEC: phase = "WARMUP"
                elif elapsed < (WARMUP_SEC + TEST_DURATION): phase = "TEST"
                else: break

                # --- INFERENCE ---
                t0 = time.time()
                # On force device=0 (GPU) et half=True (FP16) pour la performance Jetson
                model.predict(frame, imgsz=IMG_SIZE, device=0, half=True, verbose=False)
                t1 = time.time()

                # Calculs
                latence_ms = (t1 - t0) * 1000
                fps_inst = 1.0 / (t1 - t0) if (t1 - t0) > 0 else 0
                frame_count += 1

                # Log toutes les 5 frames pour ne pas ralentir
                if frame_count % 5 == 0:
                    cpu = psutil.cpu_percent()
                    ram = psutil.virtual_memory().percent
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    
                    temp = get_temp(jetson)
                    power_w = jetson.stats.get('Power TOT', 0) / 1000.0
                    
                    writer.writerow([timestamp, frame_count, f"{latence_ms:.2f}", f"{fps_inst:.2f}", cpu, ram, temp, 0.0, phase, power_w])
                    
                    if frame_count % 30 == 0:
                        print(f"\r[{phase}] FPS: {fps_inst:.1f} | Temp: {temp}°C | Pwr: {power_w:.1f}W", end="")

    cap.release()
    print(f"\n\n✅ Terminé ! Résultats : {OUTPUT_FILE}")

if __name__ == "__main__":
    run_benchmark()