import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Carregar o CSV
file_path = "finger_currents_20250306_133932.csv" 
df = pd.read_csv(file_path)

# Remover espaços extras nos nomes das colunas
df.columns = df.columns.str.strip()

# Converter valores para graus
# for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
#     df[motor] = (df[motor] * 0.229 * 2 * np.pi) / 60  # Conversão para rad/s

# Criar o gráfico
plt.figure(figsize=(10, 5))

for motor in ["Motor1", "Motor2", "Motor3", "Motor4"]:
    plt.plot(df["Timestamp"].values, df[motor].values, label=motor)  


plt.xlabel("Tempo (s)")
plt.ylabel("Corrente(mA)")
plt.title("Corrente consumida pelos motores (mA) ao Longo do Tempo")
plt.legend()
plt.grid(True)

# guardar grafico como imagem
plt.savefig("finger_currents_vel1000.png", dpi=300)

# Mostrar o gráfico
plt.show()
