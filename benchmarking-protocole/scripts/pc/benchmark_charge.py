import cv2
import time
import csv
import psutil
import subprocess
import os
from datetime import datetime
from ultralytics import YOLO

# --- CONFIGURATION ---
MODEL_PATH = "data/best.pt"
VIDEO_PATH = "data/benchmark_sequence.mp4"
# --- CHANGEMENT ICI ---
OUTPUT_DIR = "results/pc"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "charge.csv")

WARMUP_SEC = 10     
TEST_DURATION = 60 

def get_nvidia_metrics():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=temperature.gpu,power.draw", "--format=csv,noheader,nounits"],
            encoding='utf-8'
        )
        temp, power = out.strip().split(',')
        return float(temp), float(power)
    except:
        return 0.0, 0.0

def run_benchmark():
    print(f"--- MODE CHARGE (PC NVIDIA) ---")
    
    # Création du dossier spécifique
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    print("Chargement du modèle (CUDA)...")
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"❌ Erreur vidéo : {VIDEO_PATH}")
        return
    
    headers = ["Timestamp", "Frame_ID", "Latence_ms", "FPS_Inst", "CPU_%", "RAM_%", "Temp_C", "Power_Soft_W", "Phase", "Power_Manual_W"]
    
    with open(OUTPUT_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        
        start_global = time.time()
        frame_count = 0
        
        print(f"▶️ Lancement du test ({TEST_DURATION}s) vers {OUTPUT_FILE}...")
        
        while True:
            elapsed = time.time() - start_global
            success, frame = cap.read()
            if not success: break

            if elapsed < WARMUP_SEC: phase = "WARMUP"
            elif elapsed < (WARMUP_SEC + TEST_DURATION): phase = "TEST"
            else: break

            t0 = time.time()
            model.predict(frame, device=0, verbose=False, imgsz=640)
            t1 = time.time()

            latence_ms = (t1 - t0) * 1000
            fps_inst = 1.0 / (t1 - t0) if (t1 - t0) > 0 else 0
            frame_count += 1

            if frame_count % 10 == 0:
                cpu = psutil.cpu_percent()
                ram = psutil.virtual_memory().percent
                timestamp = datetime.now().strftime("%H:%M:%S")
                
                temp, power_gpu = get_nvidia_metrics()
                
                writer.writerow([timestamp, frame_count, f"{latence_ms:.2f}", f"{fps_inst:.2f}", cpu, ram, temp, power_gpu, phase, 0.0])

                if frame_count % 30 == 0:
                    print(f"\r[{phase}] FPS: {fps_inst:.1f} | GPU Pwr: {power_gpu}W", end="")

    cap.release()
    print(f"\n\n✅ Terminé ! Résultats : {OUTPUT_FILE}")

if __name__ == "__main__":
    run_benchmark()