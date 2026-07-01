import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv("surface_data.csv")
N = int(np.sqrt(len(df)))
D = df["d_obs"].values.reshape(N, N)
T = df["trav_eff"].values.reshape(N, N)
S = df["speed_scale"].values.reshape(N, N)
G = df["rep_gain"].values.reshape(N, N)

fig = plt.figure(figsize=(6, 5))
ax = fig.add_subplot(111, projection="3d")
ax.plot_surface(D, T, S, cmap="viridis", edgecolor="none")
ax.set_xlabel(r"$d_{obs}$ (m)")
ax.set_ylabel(r"$\tau_{eff}$")
ax.set_zlabel("speed_scale")
plt.savefig("fig_surface_kecepatan.pdf", bbox_inches="tight")
