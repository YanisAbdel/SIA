import psutil
import time
import csv
import platform
import subprocess
import os
from datetime import datetime

OUTPUT_FILE = "results/plancher.csv"
DURATION_SEC = 120  

def get_device_type():
    """Détecte le type de machine"""
    try:
        if os.path.exists("/etc/nv_tegra_release"):
            return "JETSON"
        subprocess.check_output(["nvidia-smi"], stderr=subprocess.STDOUT)
        return "PC_NVIDIA"
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass    
    if platform.system() == "Linux":
        return "RPI_LINUX"
    return "WINDOWS_GENERIC"

DEVICE = get_device_type()

def get_nvidia_metrics():
    """Récupère Temp et Power GPU pour PC"""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=temperature.gpu,power.draw", "--format=csv,noheader,nounits"],
            encoding='utf-8'
        )
        temp, power = out.strip().split(',')
        return float(temp), float(power)
    except:
        return 0.0, 0.0

def get_thermal_zone():
    """Récupère la température CPU sur Raspberry Pi / Jetson"""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read()) / 1000.0
    except:
        return 0.0

def run_plancher():
    print(f"--- MODE PLANCHER ---")
    print(f"Machine détectée : {DEVICE}")
    print(f"Durée de la mesure : {DURATION_SEC} secondes")
    print(f"Ne touchez a rien pendant l'enregistrement...")

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
                temp = 0.0
                power_soft = 0.0
                if DEVICE == "PC_NVIDIA":
                    temp, power_soft = get_nvidia_metrics()
                elif DEVICE in ["RPI_LINUX", "JETSON"]:
                    temp = get_thermal_zone()
                writer.writerow([timestamp, cpu, ram, temp, power_soft, 0.0])
                elapsed = int(time.time() - start_time)
                remaining = DURATION_SEC - elapsed
                print(f"\rReste : {remaining}s | CPU: {cpu}% | Temp: {temp}°C | Pwr: {power_soft}W", end="")
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\nArrêt manuel (Test incomplet).")
            return

    print(f"\n\nTerminé ! Fichier généré : {OUTPUT_FILE}")
    
    if DEVICE in ["RPI_LINUX", "JETSON"]:
        print("MESURE WATTMÈTRE USB - PLANCHER/IDLE")
        try:
            watt_value = float(input("Ecrire valeur lue sur le Wattmètre (en Watts)"))
            rows = []
            with open(OUTPUT_FILE, mode='r', newline='') as f:
                reader = csv.reader(f)
                rows = list(reader)
            for i in range(1, len(rows)):
                rows[i][5] = str(watt_value)  

            with open(OUTPUT_FILE, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(rows)
            print(f"Valeur sauvegardée : {watt_value} W dans {OUTPUT_FILE}")
        
        except ValueError:
            print("Erreur de saisie. Power_Manual_W restera à 0.")
    
    print("\nVous pouvez maintenant passer à la Phase 2 (Charge).")

if __name__ == "__main__":
    try:
        import psutil
    except ImportError:
        print("Installation de psutil...")
        os.system("pip install psutil")
        import psutil
        
    run_plancher()