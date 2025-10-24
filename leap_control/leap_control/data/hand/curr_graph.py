import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# Carregar o CSV
file_path = "test.csv"
df = pd.read_csv(file_path)


df.columns = df.columns.str.strip()

# # Converter valores para graus para as colunas de posição
# for curr_col in ["Curr1", "Curr2", "Curr3", "Curr4"]:
#     df[curr_col] = (df[curr_col] * 0.229 * 2 * np.pi) / 60  # Conversão para graus

# Identificar os MotorIDs únicos
finger_ids = df["FingerID"].unique()

# Gerar gráficos para cada MotorID
for finger_id in finger_ids:
    df_finger = df[df["FingerID"] == finger_id]  # Filtrar dados do MotorID específico
    
    plt.figure(figsize=(10, 5))
    for curr_col in ["Curr1", "Curr2", "Curr3", "Curr4"]:
        plt.plot(df_finger["Timestamp"].values, df_finger[curr_col].values, label=curr_col)
    
    plt.xlabel("Tempo (s)")
    plt.ylabel("Corrente consumida (mA)")
    plt.title(f"Corrente consumida pelos motores do dedo {finger_id} ao Longo do Tempo")
    plt.legend()
    plt.grid(True)
    
    # Salvar gráfico como imagem
    plt.savefig(f"finger_curr4_finger_{finger_id}.png", dpi=300)
    
    # Mostrar o gráfico
    plt.show()
