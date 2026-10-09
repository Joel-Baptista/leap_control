import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


R_values = [0.5 ,1, 2, 3.3, 6, 10, 22]
b  = 2.466      
Vin = 3.3       
m  = -0.8103        

palette = sns.color_palette("Dark2", n_colors=len(R_values)) 

def F_of_Vout(Vout, R, b, Vin, m):
    Vout = np.asarray(Vout, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        base = (R / (10.0 ** b)) * ((Vin - Vout) / Vout)

        base = np.where(base >= 0, base, np.nan)
        F = 0.00980665 * (base ** (1.0 / m))
    return F

VOUT_MAX = 3.3
F_MIN, F_MAX = 0, 100

eps = 1e-6
line_rows = []
scatter_rows = []

for R in R_values:
    # Continuous curve (dense sampling)
    vout_line = np.linspace(eps, VOUT_MAX, 5000)
    f_line = F_of_Vout(vout_line, R, b, Vin, m)

    # Scatter with 1024 evenly spaced points in Vout
    vout_scatter = np.linspace(eps, VOUT_MAX, 1024)
    f_sc = F_of_Vout(vout_scatter, R, b, Vin, m)

    line_rows.append(pd.DataFrame({
        "Vout": vout_line,
        "F": np.clip(f_line, F_MIN, F_MAX),
        "R": str(R)
    }))
    scatter_rows.append(pd.DataFrame({
        "Vout": vout_scatter,
        "F": np.clip(f_sc, F_MIN, F_MAX),
        "R": str(R)
    }))

    # DataFrames (nice for seaborn)
df_line = pd.concat(line_rows, ignore_index=True)
df_scatter = pd.concat(scatter_rows, ignore_index=True)

# -----------------------------
# Plotting (Seaborn)
# -----------------------------
sns.set_theme(style="whitegrid")

fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)

# Continuous plot: multiple R curves
sns.lineplot(data=df_line, x="Vout", y="F", hue="R",  palette=palette, ax=axes[0])
axes[0].set_title("Continuous (multiple R)")
axes[0].set_xlim(0, VOUT_MAX)
axes[0].set_ylim(F_MIN, F_MAX)
axes[0].set_xlabel("Vout (V)")
axes[0].set_ylabel("F (N)")
axes[0].legend(title="R", loc="best")

# Scatter plot: multiple R point clouds
sns.scatterplot(data=df_scatter, x="Vout", y="F", hue="R",  palette=palette, s=12, ax=axes[1])
axes[1].set_title("Scatter (1024 points, multiple R)")
axes[1].set_xlim(0, VOUT_MAX)
axes[1].set_ylim(F_MIN, F_MAX)
axes[1].set_xlabel("Vout (V)")
axes[1].set_ylabel("F (N)")
axes[1].legend(title="R", loc="best")

plt.show()

