import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# Carregar o CSV
file_path = "release.csv"
df = pd.read_csv(file_path)


df.columns = df.columns.str.strip()

# Converter valores para graus para as colunas de posição
for vel_col in ["Vel1", "Vel2", "Vel3", "Vel4"]:
    df[vel_col] = (df[vel_col] * 0.229 * 2 * np.pi) / 60  # Conversão para graus

# Identificar os MotorIDs únicos
finger_ids = df["FingerID"].unique()

# Gerar gráficos para cada MotorID
for finger_id in finger_ids:
    df_finger = df[df["FingerID"] == finger_id]  # Filtrar dados do MotorID específico
    
    plt.figure(figsize=(10, 5))
    for vel_col in ["Vel1", "Vel2", "Vel3", "Vel4"]:
        plt.plot(df_finger["Timestamp"].values, df_finger[vel_col].values, label=vel_col)
    
    plt.xlabel("Tempo (s)")
    plt.ylabel("Velocidade (rad/s)")
    plt.title(f"Velocidade dos motores do dedo {finger_id} ao Longo do Tempo")
    plt.legend()
    plt.grid(True)
    
    # Salvar gráfico como imagem
    plt.savefig(f"finger_vel4_finger_{finger_id}.png", dpi=300)
    
    # Mostrar o gráfico
    plt.show()
