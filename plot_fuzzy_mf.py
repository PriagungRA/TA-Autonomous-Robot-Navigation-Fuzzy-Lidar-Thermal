#!/usr/bin/env python3
# plot_fuzzy_mf_fixed.py
# Verifikasi dan plot fungsi keanggotaan sesuai fuzzy_controller.h
# Fix: fungsi dapat menerima float atau array

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# ------------------------------------------------------------
# Fungsi keanggotaan yang dapat menerima float atau array
# ------------------------------------------------------------
def trimf(x, a, b, c):
    x = np.asarray(x)
    y = np.zeros_like(x, dtype=float)
    # Naik
    mask = (x > a) & (x < b)
    y[mask] = (x[mask] - a) / (b - a)
    # Puncak
    y[x == b] = 1.0
    # Turun
    mask = (x > b) & (x < c)
    y[mask] = (c - x[mask]) / (c - b)
    return y

def trapmf(x, a, b, c, d):
    x = np.asarray(x)
    y = np.zeros_like(x, dtype=float)
    # Naik
    mask = (x > a) & (x < b)
    y[mask] = (x[mask] - a) / (b - a)
    # Datar di puncak
    y[(x >= b) & (x <= c)] = 1.0
    # Turun
    mask = (x > c) & (x < d)
    y[mask] = (d - x[mask]) / (d - c)
    return y

# ------------------------------------------------------------
# Parameter persis dari fuzzy_controller.h
# ------------------------------------------------------------
# Jarak
def j_dekat(x): return trapmf(x, 0.0, 0.0, 0.30, 0.60)
def j_sedang(x): return trimf(x, 0.40, 0.80, 1.20)
def j_jauh(x): return trapmf(x, 1.00, 1.40, 2.00, 2.00)

# Traversability
def t_rendah(x): return trapmf(x, 0.0, 0.0, 0.25, 0.45)
def t_sedang(x): return trimf(x, 0.35, 0.55, 0.75)
def t_tinggi(x): return trapmf(x, 0.60, 0.80, 1.00, 1.00)

# Speed
def s_lambat(x): return trapmf(x, 0.0, 0.0, 0.15, 0.35)
def s_sedang(x): return trimf(x, 0.30, 0.50, 0.70)
def s_cepat(x): return trapmf(x, 0.60, 0.80, 1.00, 1.00)

# Gain
def g_lemah(x): return trapmf(x, 0.5, 0.5, 0.9, 1.3)
def g_sedang(x): return trimf(x, 1.1, 1.5, 1.9)
def g_kuat(x): return trapmf(x, 1.7, 2.1, 2.5, 2.5)

# ------------------------------------------------------------
# Verifikasi nilai pada titik-titik kritis (sekarang bisa float)
# ------------------------------------------------------------
print("=== VERIFIKASI FUNGSI KEANGGOTAAN ===")
print(f"Jarak - dekat(0.0)={j_dekat(0.0):.1f}, dekat(0.3)={j_dekat(0.3):.1f}, dekat(0.6)={j_dekat(0.6):.1f}")
print(f"Jarak - sedang(0.4)={j_sedang(0.4):.1f}, sedang(0.8)={j_sedang(0.8):.1f}, sedang(1.2)={j_sedang(1.2):.1f}")
print(f"Jarak - jauh(1.0)={j_jauh(1.0):.1f}, jauh(1.4)={j_jauh(1.4):.1f}, jauh(2.0)={j_jauh(2.0):.1f}")
print(f"Gain - lemah(0.5)={g_lemah(0.5):.1f}, lemah(1.3)={g_lemah(1.3):.1f}")
print(f"Gain - kuat(1.7)={g_kuat(1.7):.1f}, kuat(2.1)={g_kuat(2.1):.1f}, kuat(2.5)={g_kuat(2.5):.1f}")

# ------------------------------------------------------------
# Plot dengan resolusi tinggi
# ------------------------------------------------------------
x_jarak = np.linspace(0, 2, 1000)
x_trav = np.linspace(0, 1, 1000)
x_speed = np.linspace(0, 1, 1000)
x_gain = np.linspace(0.5, 2.5, 1000)

def setup_ax(ax, title, xlabel, ylabel, xlim, ylim=(0,1.05)):
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.grid(True, linestyle=':', alpha=0.6, linewidth=0.5)
    ax.xaxis.set_minor_locator(MultipleLocator(0.1))
    ax.tick_params(axis='both', which='major', labelsize=9)

# Gambar 1: Jarak
fig1, ax1 = plt.subplots(figsize=(5,3))
setup_ax(ax1, 'Jarak Obstacle ($d_{\\mathrm{obs}}$)',
         '$d_{\\mathrm{obs}}$ (m)', '$\\mu$', (0,2))
ax1.plot(x_jarak, j_dekat(x_jarak), 'k-', label='Dekat', linewidth=1.8)
ax1.plot(x_jarak, j_sedang(x_jarak), 'k--', label='Sedang', linewidth=1.8)
ax1.plot(x_jarak, j_jauh(x_jarak), 'k-.', label='Jauh', linewidth=1.8)
ax1.legend(loc='upper right', fontsize=9)
ax1.annotate('0.40', xy=(0.40, 1.02), ha='center', fontsize=8, color='gray')
ax1.annotate('0.80', xy=(0.80, 1.02), ha='center', fontsize=8, color='gray')
ax1.annotate('1.20', xy=(1.20, 1.02), ha='center', fontsize=8, color='gray')
plt.tight_layout()
plt.savefig('fig_mf_jarak.pdf', dpi=300)
plt.savefig('fig_mf_jarak.png', dpi=300)
plt.close()

# Gambar 2: Traversability
fig2, ax2 = plt.subplots(figsize=(5,3))
setup_ax(ax2, 'Traversability Efektif ($\\tau_{\\mathrm{eff}}$)',
         '$\\tau_{\\mathrm{eff}}$', '$\\mu$', (0,1))
ax2.plot(x_trav, t_rendah(x_trav), 'k-', label='Rendah', linewidth=1.8)
ax2.plot(x_trav, t_sedang(x_trav), 'k--', label='Sedang', linewidth=1.8)
ax2.plot(x_trav, t_tinggi(x_trav), 'k-.', label='Tinggi', linewidth=1.8)
ax2.legend(loc='upper right', fontsize=9)
ax2.annotate('0.35', xy=(0.35, 1.02), ha='center', fontsize=8, color='gray')
ax2.annotate('0.55', xy=(0.55, 1.02), ha='center', fontsize=8, color='gray')
ax2.annotate('0.75', xy=(0.75, 1.02), ha='center', fontsize=8, color='gray')
plt.tight_layout()
plt.savefig('fig_mf_trav.pdf', dpi=300)
plt.savefig('fig_mf_trav.png', dpi=300)
plt.close()

# Gambar 3: Speed scale
fig3, ax3 = plt.subplots(figsize=(5,3))
setup_ax(ax3, 'Skala Kecepatan ($\\mathrm{speed\\_scale}$)',
         '$\\mathrm{speed\\_scale}$', '$\\mu$', (0,1))
ax3.plot(x_speed, s_lambat(x_speed), 'k-', label='Lambat', linewidth=1.8)
ax3.plot(x_speed, s_sedang(x_speed), 'k--', label='Sedang', linewidth=1.8)
ax3.plot(x_speed, s_cepat(x_speed), 'k-.', label='Cepat', linewidth=1.8)
ax3.legend(loc='upper right', fontsize=9)
ax3.annotate('0.30', xy=(0.30, 1.02), ha='center', fontsize=8, color='gray')
ax3.annotate('0.50', xy=(0.50, 1.02), ha='center', fontsize=8, color='gray')
ax3.annotate('0.70', xy=(0.70, 1.02), ha='center', fontsize=8, color='gray')
plt.tight_layout()
plt.savefig('fig_mf_kecepatan.pdf', dpi=300)
plt.savefig('fig_mf_kecepatan.png', dpi=300)
plt.close()

# Gambar 4: Repulsive gain
fig4, ax4 = plt.subplots(figsize=(5,3))
setup_ax(ax4, 'Penguatan Repulsi ($\\mathrm{rep\\_gain}$)',
         '$\\mathrm{rep\\_gain}$', '$\\mu$', (0.5,2.5))
ax4.plot(x_gain, g_lemah(x_gain), 'k-', label='Lemah', linewidth=1.8)
ax4.plot(x_gain, g_sedang(x_gain), 'k--', label='Sedang', linewidth=1.8)
ax4.plot(x_gain, g_kuat(x_gain), 'k-.', label='Kuat', linewidth=1.8)
ax4.legend(loc='upper right', fontsize=9)
ax4.annotate('1.10', xy=(1.10, 1.02), ha='center', fontsize=8, color='gray')
ax4.annotate('1.50', xy=(1.50, 1.02), ha='center', fontsize=8, color='gray')
ax4.annotate('1.90', xy=(1.90, 1.02), ha='center', fontsize=8, color='gray')
plt.tight_layout()
plt.savefig('fig_mf_gain.pdf', dpi=300)
plt.savefig('fig_mf_gain.png', dpi=300)
plt.close()

print("\nGambar telah dihasilkan ulang dengan verifikasi parameter.")
print("Periksa output verifikasi di atas. Nilai harus sesuai:")
print("- dekat(0.3)=1.0, dekat(0.6)=0.0")
print("- jauh(1.0)=0.0, jauh(1.4)=1.0")
