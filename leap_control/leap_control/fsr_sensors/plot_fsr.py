# plot_fsr_data.py
import pandas as pd
import matplotlib.pyplot as plt

# Altere o nome do ficheiro conforme necessário
csv_file = 'fsr_data_20250410_1530.csv'  # <- substitui pelo teu ficheiro real

# Carregar os dados
df = pd.read_csv(csv_file, parse_dates=['timestamp'])

# Gráfico
plt.figure(figsize=(12, 6))
for i in range(1, 5):
    plt.plot(df['timestamp'], df[f'FSR{i}'], label=f'FSR {i}')

plt.title('Leituras dos sensores FSR ao longo do tempo')
plt.xlabel('Tempo')
plt.ylabel('Valor do sensor')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
