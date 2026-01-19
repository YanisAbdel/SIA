import csv
import os

# --- CHANGEMENTS ICI ---
INPUT_DIR = "results/pc"
FILE_IDLE = os.path.join(INPUT_DIR, "plancher.csv")
FILE_LOAD = os.path.join(INPUT_DIR, "charge.csv")
REPORT_FILE = os.path.join(INPUT_DIR, "rapport_final.txt")
REPORT_CSV_FILE = os.path.join(INPUT_DIR, "rapport_final.csv")

def load_data(filepath):
    data = []
    if not os.path.exists(filepath):
        print(f"ERREUR : Le fichier {filepath} est introuvable.")
        return None
    with open(filepath, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    return data

def get_avg(data, key, filter_phase=None):
    values = []
    for row in data:
        if filter_phase and row.get("Phase") != filter_phase:
            continue    
        try:
            val = float(row[key])
            if val > 0: values.append(val)
        except: continue
    return sum(values) / len(values) if values else 0.0

def main():
    print(f"--- ANALYSEUR DE RÉSULTATS (Dossier: {INPUT_DIR}) ---")
    
    idle_data = load_data(FILE_IDLE)
    load_data_raw = load_data(FILE_LOAD)
    
    if not idle_data or not load_data_raw: return

    avg_temp_idle = get_avg(idle_data, "Temp_C")
    avg_pwr_soft_idle = get_avg(idle_data, "Power_Soft_W")
    
    avg_fps = get_avg(load_data_raw, "FPS_Inst", filter_phase="TEST")
    avg_lat = get_avg(load_data_raw, "Latence_ms", filter_phase="TEST")
    avg_temp_load = get_avg(load_data_raw, "Temp_C", filter_phase="TEST")
    avg_pwr_soft_load = get_avg(load_data_raw, "Power_Soft_W", filter_phase="TEST")

    final_pwr_idle = avg_pwr_soft_idle
    final_pwr_load = avg_pwr_soft_load
    source_pwr = "LOGICIELLE (Nvidia Power Draw)"

    print(f"Puissance détectée : {source_pwr}")

    delta_watts = final_pwr_load - final_pwr_idle
    efficiency = avg_fps / final_pwr_load if final_pwr_load > 0 else 0
    delta_temp = avg_temp_load - avg_temp_idle

    report = f"""
===================================================
        RAPPORT FINAL (PC NVIDIA)
===================================================
1. PERFORMANCES
---------------------
FPS Moyen       : {avg_fps:.2f} fps
Latence Moyenne : {avg_lat:.2f} ms

2. THERMIQUE (GPU)
------------------
Temp. Idle      : {avg_temp_idle:.1f} °C
Temp. Charge    : {avg_temp_load:.1f} °C
Augmentation    : +{delta_temp:.1f} °C

3. ÉNERGIE (GPU Only)
---------------------
Conso Idle      : {final_pwr_idle:.2f} W
Conso Charge    : {final_pwr_load:.2f} W
DELTA IA        : {delta_watts:.2f} W

EFFICACITÉ GLOBALE : {efficiency:.2f} FPS/Watt
===================================================
"""
    print(report)
    
    with open(REPORT_FILE, "w") as f: f.write(report)
    
    csv_headers = ["Source", "FPS", "Latence", "Temp_Idle", "Temp_Charge", "Conso_Idle", "Conso_Charge", "Efficiency"]
    csv_data = [source_pwr, f"{avg_fps:.2f}", f"{avg_lat:.2f}", f"{avg_temp_idle:.1f}", f"{avg_temp_load:.1f}", f"{final_pwr_idle:.2f}", f"{final_pwr_load:.2f}", f"{efficiency:.2f}"]
    
    with open(REPORT_CSV_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)
        writer.writerow(csv_data)
    
    print(f"Rapports sauvegardés dans {INPUT_DIR}")

if __name__ == "__main__":
    main()