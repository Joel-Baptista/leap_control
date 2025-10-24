# plot_fsr.py
import pandas as pd
import matplotlib.pyplot as plt

# Caminho do CSV
csv_file = 'test.csv'

# Carrega o DataFrame
df = pd.read_csv(csv_file)

# Garante que todos os dados são numéricos
df = df.apply(pd.to_numeric, errors='coerce')

# Lista de colunas FSR
fsr_cols = [col for col in df.columns if col.startswith('FSR')]

# Plotagem
plt.figure(figsize=(12, 6))
for col in fsr_cols:
    plt.plot(df['timestamp'].values, df[col].values, label=col)

plt.title('Leituras dos sensores FSR ao longo do tempo')
plt.xlabel('Tempo (s)')
plt.ylabel('Valor do sensor (V)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Guardar o gráfico
plt.savefig('test.png')  # podes mudar o nome e formato, ex: .pdf, .svg, etc

# Mostrar o gráfico
plt.show()
