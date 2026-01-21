import psutil
import time
import csv
import subprocess
import os
from datetime import datetime

# --- CHANGEMENT ICI ---
OUTPUT_DIR = "results/pc"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "plancher.csv")
DURATION_SEC = 60  

def get_nvidia_metrics():
    """Récupère Température et Conso GPU via nvidia-smi"""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=temperature.gpu,power.draw", "--format=csv,noheader,nounits"],
            encoding='utf-8'
        )
        temp, power = out.strip().split(',')
        return float(temp), float(power)
    except Exception as e:
        return 0.0, 0.0

def run_plancher():
    print(f"--- MODE PLANCHER (PC NVIDIA) ---")
    print(f"Destination : {OUTPUT_FILE}")
    print(f"Durée : {DURATION_SEC} secondes")

    # Création du dossier spécifique
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    headers = ["Timestamp", "CPU_Load_%", "RAM_Load_%", "Temp_C", "Power_Soft_W", "Power_Manual_W"]

    with open(OUTPUT_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        
        start_time = time.time()
        try:
            while (time.time() - start_time) < DURATION_SEC:
                cpu = psutil.cpu_percent(interval=0.1)
                ram = psutil.virtual_memory().percent
                timestamp = datetime.now().strftime("%H:%M:%S")        
                
                temp, power_gpu = get_nvidia_metrics()

                writer.writerow([timestamp, cpu, ram, temp, power_gpu, 0.0])
                
                remaining = int(DURATION_SEC - (time.time() - start_time))
                print(f"\rReste : {remaining}s | CPU: {cpu}% | GPU Pwr: {power_gpu}W", end="")
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\nArrêt manuel.")

    print(f"\n✅ Terminé ! Fichier : {OUTPUT_FILE}")

if __name__ == "__main__":
    run_plancher()