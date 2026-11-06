#!/usr/bin/env python3
"""
graficar_vueltas.py - Figuras de la odometría a partir del CSV de vueltas_odom.py.

Uso:
    python3 graficar_vueltas.py resultados/vueltas_XXXX.csv --titulo "Pista propia"

Genera dos figuras (PNG junto al CSV):
    1) Trayectoria real (/pose_real) y según la odometría (odom_kinematic), coloreada por vuelta.
    2) Error de posición y error de rumbo ACUMULADO (sin envolver) en función del tiempo,
       con el valor al cerrar cada vuelta.

Columna 'vuelta' del CSV = número de cruces de la línea ya registrados:
    0 = antes de la salida, 1 = vuelta 1 en curso, 2 = vuelta 2, 3 = vuelta 3.
    El nodo deja de escribir justo al cerrar la última vuelta.
"""

import argparse
import csv
import os

import numpy as np
import matplotlib.pyplot as plt


def leer_csv(ruta):
    with open(ruta, newline='') as f:
        filas = list(csv.DictReader(f))
    if not filas:
        raise SystemExit(f'CSV vacío: {ruta}')
    d = {k: np.array([float(fl[k]) for fl in filas]) for k in filas[0]}
    d['vuelta'] = d['vuelta'].astype(int)
    return d


def graficar(ruta, titulo, n_vueltas, mostrar):
    d = leer_csv(ruta)
    t = d['t']
    v = d['vuelta']

    # Rumbo sin envolver: np.unwrap corrige los saltos de ±2π.
    # Es válido porque a 10 Hz el giro entre dos muestras es < π (ω_odom máx ≈ 5.8 rad/s → 0.58 rad).
    psi_r = np.unwrap(d['psi_real'])
    psi_o = np.unwrap(d['psi_odom'])
    e_psi = np.degrees(psi_o - psi_r)
    e_pos = np.hypot(d['x_odom'] - d['x_real'], d['y_odom'] - d['y_real'])

    # Cierres de vuelta: primera muestra en que el contador pasa a 2 y 3 (fin de vueltas 1 y 2),
    # más la última muestra si la carrera terminó (fin de la última vuelta).
    cambios = np.flatnonzero(np.diff(v) > 0) + 1
    cierres = [i for i in cambios if 2 <= v[i] <= n_vueltas + 1]
    # El nodo escribe la fila del CSV después de registrar el cruce: la fila con vuelta = n+1
    # puede existir (cierre ya detectado arriba) o no (1 de cada 3 muestras). Solo se añade
    # la última muestra si falta el cierre de la última vuelta.
    if v[-1] >= n_vueltas and len(cierres) < n_vueltas:
        cierres.append(len(v) - 1)

    base = os.path.splitext(ruta)[0]
    colores = plt.cm.tab10.colors

    # ---------------- Figura 1: trayectorias ----------------
    fig1, ax = plt.subplots(figsize=(10, 6))
    ax.plot(d['x_real'], d['y_real'], 'k-', lw=2.5, label='Real (/pose_real, Gazebo)')
    for k in range(0, n_vueltas + 1):
        m = v == k
        if not m.any():
            continue
        etiqueta = 'Odometría antes de la salida' if k == 0 else f'Odometría vuelta {k}'
        ax.plot(d['x_odom'][m], d['y_odom'][m], '--', lw=1.5,
                color='gray' if k == 0 else colores[k - 1], label=etiqueta)
    ax.plot(d['x_real'][0], d['y_real'][0], 'ko', ms=8, label='Spawn')
    ax.axvline(0.0, color='green', ls=':', lw=1.5, label='Línea de meta (x = 0)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{titulo}: trayectoria real vs odometría ({n_vueltas} vueltas)')
    ax.legend(loc='best', fontsize=9)
    fig1.tight_layout()
    fig1.savefig(base + '_trayectoria.png', dpi=150)

    # ---------------- Figura 2: errores en función del tiempo ----------------
    fig2, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    a1.plot(t, e_pos, 'b-')
    a1.set_ylabel('Error de posición [m]')
    a2.plot(t, e_psi, 'r-')
    a2.set_ylabel('Error de rumbo acumulado [°]')
    a2.set_xlabel('Tiempo simulado /clock [s]')
    for n, i in enumerate(cierres, start=1):
        for a, y, fmt in ((a1, e_pos[i], f'{e_pos[i]:.2f} m'), (a2, e_psi[i], f'{e_psi[i]:+.1f}°')):
            a.axvline(t[i], color='gray', ls=':')
            a.plot(t[i], y, 'ko')
            a.annotate(f'V{n}: {fmt}', (t[i], y), textcoords='offset points',
                       xytext=(-60, 8), fontsize=9)
    a1.grid(True, alpha=0.3)
    a2.grid(True, alpha=0.3)
    a1.set_title(f'{titulo}: error de la odometría (cierre de cada vuelta)')
    fig2.tight_layout()
    fig2.savefig(base + '_errores.png', dpi=150)

    # ---------------- Resumen en la terminal ----------------
    print(f'\n{titulo}  ({os.path.basename(ruta)})')
    print(' Vuelta | e_pos [m] | e_psi acumulado [°] | incremento [°]')
    previo = 0.0
    for n, i in enumerate(cierres, start=1):
        print(f'   {n}    |  {e_pos[i]:7.3f}  |      {e_psi[i]:+8.1f}       |   {e_psi[i] - previo:+7.1f}')
        previo = e_psi[i]
    print(f' Figuras: {base}_trayectoria.png  y  {base}_errores.png')

    if mostrar:
        plt.show()
    plt.close('all')


def main():
    p = argparse.ArgumentParser(description='Gráficas de vueltas y error de odometría')
    p.add_argument('csv', help='CSV generado por vueltas_odom.py')
    p.add_argument('--titulo', default='Pista', help='Título de las figuras')
    p.add_argument('--n_vueltas', type=int, default=3)
    p.add_argument('--sin_mostrar', action='store_true', help='Solo guarda los PNG')
    a = p.parse_args()
    graficar(a.csv, a.titulo, a.n_vueltas, not a.sin_mostrar)


if __name__ == '__main__':
    main()
