import pandas as pd
import matplotlib.pyplot as plt

# Carregar o CSV
file_path = "finger_positions_20250306_145242.csv" 
df = pd.read_csv(file_path)

# Remover espaços extras nos nomes das colunas
df.columns = df.columns.str.strip()

# Converter valores para graus
for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
    df[motor] = (df[motor] / 4095) * 360  # Conversão para graus

# Criar o gráfico
plt.figure(figsize=(10, 5))

for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
    plt.plot(df["Timestamp"].values, df[motor].values, label=motor)  


plt.xlabel("Tempo (s)")
plt.ylabel("Posição (°)")
plt.title("Posição dos Motores (graus) ao Longo do Tempo")
plt.legend()
plt.grid(True)

# guardar grafico como imagem
plt.savefig("finger_pos_vel50_obs_motor2.png", dpi=300)

# Mostrar o gráfico
plt.show()
