import pandas as pd
import matplotlib.pyplot as plt

# Carregar o CSV
file_path = "release.csv"
df = pd.read_csv(file_path)


df.columns = df.columns.str.strip()

# Converter valores para graus para as colunas de posição
for pos_col in ["Pos1", "Pos2", "Pos3", "Pos4"]:
    df[pos_col] = (df[pos_col] / 4095) * 360  # Conversão para graus
    

# Identificar os MotorIDs únicos
finger_ids = df["FingerID"].unique()

# Gerar gráficos para cada MotorID
for finger_id in finger_ids:
    df_finger = df[df["FingerID"] == finger_id]  # Filtrar dados do MotorID específico
    
    plt.figure(figsize=(10, 5))
    for pos_col in ["Pos1", "Pos2", "Pos3", "Pos4"]:
        plt.plot(df_finger["Timestamp"].values, df_finger[pos_col].values, label=pos_col)
    
    plt.xlabel("Tempo (s)")
    plt.ylabel("Posição (°)")
    plt.title(f"Posição do dedo {finger_id} ao Longo do Tempo")
    plt.legend()
    plt.grid(True)
    
    # Salvar gráfico como imagem
    plt.savefig(f"finger_pos4_finger_{finger_id}.png", dpi=300)
    
    # Mostrar o gráfico
    plt.show()
