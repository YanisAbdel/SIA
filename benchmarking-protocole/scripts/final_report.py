import pandas as pd
import matplotlib.pyplot as plt
import os

# --- CONFIGURATION ---
reports = {
    "Raspberry Pi 4": "results/pi4/rapport_final.csv",
    "Jetson Orin Nano": "results/jetson/rapport_final.csv",
    "PC Portable (Nvidia)": "results/pc/rapport_final.csv"
}

OUTPUT_DIR = "results/final"
MASTER_CSV = os.path.join(OUTPUT_DIR, "comparatif_global.csv")
MASTER_PNG = os.path.join(OUTPUT_DIR, "graphique_comparatif.png")

def generate_master():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    combined_data = []

    print("--- FUSION DES DONNÉES ---")
    for device, path in reports.items():
        if os.path.exists(path):
            try:
                df = pd.read_csv(path)
                
                # NETTOYAGE DES ESPACES (casse-pieds habituel)
                df.columns = df.columns.str.strip()
                
                # Dictionnaire de traduction (PC -> Standard)
                # Si le PC utilise "FPS", on le renomme "FPS_Moyen" pour s'aligner
                rename_map = {
                    "FPS": "FPS_Moyen",
                    "Latence": "Latence_ms",
                    "Temp_Idle": "Temp_Idle_C",
                    "Temp_Charge": "Temp_Charge_C",
                    "Conso_Idle": "Conso_Idle_W",
                    "Conso_Charge": "Conso_Charge_W",
                    "Efficiency": "Efficacite_FPS_per_W",
                    "Source": "Source_Puissance"
                }
                df = df.rename(columns=rename_map)
                
                # Ajout de la colonne Nom de l'appareil
                df.insert(0, 'Device', device)
                
                # On garde seulement les colonnes utiles pour éviter le bazar
                cols_to_keep = ['Device', 'FPS_Moyen', 'Latence_ms', 'Temp_Charge_C', 'Conso_Charge_W', 'Efficacite_FPS_per_W']
                # On filtre pour ne garder que celles qui existent vraiment
                existing_cols = [c for c in cols_to_keep if c in df.columns]
                df_clean = df[existing_cols]

                combined_data.append(df_clean)
                print(f"✅ {device} : OK ({len(df)} ligne)")
                
            except Exception as e:
                print(f"❌ Erreur {device}: {e}")
        else:
            print(f"⚠️ Manquant : {path}")

    if not combined_data:
        return

    # 1. Création du CSV Master Propre
    master_df = pd.concat(combined_data, ignore_index=True)
    master_df.to_csv(MASTER_CSV, index=False)
    print(f"\n📄 Tableau corrigé sauvegardé : {MASTER_CSV}")

    # 2. Génération du Graphique
    try:
        # Conversion numérique (sécurité)
        master_df['FPS'] = pd.to_numeric(master_df['FPS_Moyen'])
        master_df['Efficiency'] = pd.to_numeric(master_df['Efficacite_FPS_per_W'])

        # Création de la figure
        fig, ax1 = plt.subplots(figsize=(12, 7))

        # --- BARRES (FPS) ---
        color_fps = '#1f77b4' # Bleu
        ax1.set_xlabel('Matériel', fontweight='bold')
        ax1.set_ylabel('Vitesse (FPS)', color=color_fps, fontweight='bold', fontsize=12)
        
        bars = ax1.bar(master_df['Device'], master_df['FPS'], color=color_fps, alpha=0.7, width=0.5, label='FPS')
        ax1.tick_params(axis='y', labelcolor=color_fps)
        ax1.grid(axis='y', linestyle='--', alpha=0.3)

        # Ajout des valeurs sur les barres
        for bar in bars:
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + 1,
                     f'{int(height)} fps', ha='center', va='bottom', color=color_fps, fontweight='bold')

        # --- LIGNE (EFFICACITÉ) ---
        ax2 = ax1.twinx()
        color_eff = '#d62728' # Rouge
        ax2.set_ylabel('Efficacité (FPS / Watt)', color=color_eff, fontweight='bold', fontsize=12)
        
        line = ax2.plot(master_df['Device'], master_df['Efficiency'], color=color_eff, marker='o', linewidth=3, markersize=10, label='Efficacité')
        ax2.tick_params(axis='y', labelcolor=color_eff)

        # Ajout des valeurs sur les points
        for i, txt in enumerate(master_df['Efficiency']):
            ax2.annotate(f"{txt:.2f}", (i, master_df['Efficiency'][i]), 
                         xytext=(0, 10), textcoords='offset points', ha='center', color=color_eff, fontweight='bold')

        plt.title('LE VERDICT : Puissance vs Intelligence Énergétique', fontsize=14, pad=20)
        plt.tight_layout()
        
        plt.savefig(MASTER_PNG)
        print(f"📊 Graphique généré : {MASTER_PNG}")
        plt.show()

    except Exception as e:
        print(f"❌ Erreur graphique : {e}")

if __name__ == "__main__":
    generate_master()