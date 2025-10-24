import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Carregar o CSV
file_path = "finger_velocities_20250306_150453.csv" 
df = pd.read_csv(file_path)

# Remover espaços extras nos nomes das colunas
df.columns = df.columns.str.strip()

# Converter valores para graus
for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
    df[motor] = (df[motor] * 0.229 * 2 * np.pi) / 60  # Conversão para rad/s

# Criar o gráfico
plt.figure(figsize=(10, 5))

for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
    plt.plot(df["Timestamp"].values, df[motor].values, label=motor)  


plt.xlabel("Tempo (s)")
plt.ylabel("Velocidade (rad/s)")
plt.title("Velocidade dos Motores (rad/s) ao Longo do Tempo")
plt.legend()
plt.grid(True)

# guardar grafico como imagem
plt.savefig("finger_vels_vel50_obs.png", dpi=300)

# Mostrar o gráfico
plt.show()
