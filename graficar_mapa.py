#!/usr/bin/env python3
"""
graficar_mapa.py
Genera una imagen gráfica (PNG) del laberinto
"""

import json
import argparse
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def resolver_ruta_optima(rows, cols, walls, start, goals):
    """Calcula la ruta más corta (BFS) desde el inicio hasta la meta."""
    dirs = [
        (-1, 0, 'top'),
        (0, 1, 'right'),
        (1, 0, 'bottom'),
        (0, -1, 'left')
    ]
    queue = [(start[0], start[1], [start])]
    visited = {start}
    
    while queue:
        r, c, path = queue.pop(0)
        if (r, c) in goals:
            return path
        for dr, dc, wall_key in dirs:
            if not walls[r][c][wall_key]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc, path + [(nr, nc)]))
    return None


def graficar_laberinto(json_path, salida_png=None, mostrar=False, dibujar_ruta=True):
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    rows = data['rows']
    cols = data['cols']
    walls = data['walls']
    start_r = data['start']['r']
    start_c = data['start']['c']
    cell_cm = data.get('geometry', {}).get('cellCm', 18)

    goals = {
        ((rows - 1) // 2, (cols - 1) // 2),
        ((rows - 1) // 2, cols // 2),
        (rows // 2, (cols - 1) // 2),
        (rows // 2, cols // 2)
    }

    fig, ax = plt.subplots(figsize=(10, 10), dpi=200)

    ax.set_facecolor('#F8F9FA')

    
    def to_plot_xy(r, c):
        return c, (rows - 1 - r)

    # Sombrear casilla de inicio
    sc_x, sc_y = to_plot_xy(start_r, start_c)
    ax.add_patch(patches.Rectangle((sc_x, sc_y), 1, 1, color='#D4EDDA', zorder=1))
    ax.text(sc_x + 0.5, sc_y + 0.5, 'S', color='#155724', fontsize=14,
            fontweight='bold', ha='center', va='center', zorder=2)

    #Sombrea zona de meta
    for gr, gc in goals:
        gx, gy = to_plot_xy(gr, gc)
        ax.add_patch(patches.Rectangle((gx, gy), 1, 1, color='#FFF3CD', zorder=1))
        ax.text(gx + 0.5, gy + 0.5, 'G', color='#856404', fontsize=12,
                fontweight='bold', ha='center', va='center', zorder=2)

    # Dibuja ruta óptima si está habilitada
    if dibujar_ruta:
        path = resolver_ruta_optima(rows, cols, walls, (start_r, start_c), goals)
        if path and len(path) > 1:
            px = [to_plot_xy(r, c)[0] + 0.5 for r, c in path]
            py = [to_plot_xy(r, c)[1] + 0.5 for r, c in path]
            ax.plot(px, py, color='#007BFF', linestyle='--', linewidth=2.5,
                    marker='o', markersize=4, label=f'Ruta óptima ({len(path)-1} casillas)', zorder=3)

    # Dibuja paredes
    wall_color = '#1A1A1A'
    wall_width = 3.0

    for r in range(rows):
        for c in range(cols):
            x, y = to_plot_xy(r, c)
            cell_walls = walls[r][c]

            # Pared Norte (arriba)
            if cell_walls['top']:
                ax.plot([x, x + 1], [y + 1, y + 1], color=wall_color, linewidth=wall_width, solid_capstyle='round', zorder=4)
            # Pared Sur (abajo)
            if cell_walls['bottom']:
                ax.plot([x, x + 1], [y, y], color=wall_color, linewidth=wall_width, solid_capstyle='round', zorder=4)
            # Pared Oeste (izquierda)
            if cell_walls['left']:
                ax.plot([x, x], [y, y + 1], color=wall_color, linewidth=wall_width, solid_capstyle='round', zorder=4)
            # Pared Este (derecha)
            if cell_walls['right']:
                ax.plot([x + 1, x + 1], [y, y + 1], color=wall_color, linewidth=wall_width, solid_capstyle='round', zorder=4)

    #Dibuja postes
    for x in range(cols + 1):
        for y in range(rows + 1):
            ax.plot(x, y, marker='s', color='#DC3545', markersize=3, zorder=5)

    # Configuración de ejes y cuadrícula
    ax.set_xlim(-0.2, cols + 0.2)
    ax.set_ylim(-0.2, rows + 0.2)
    ax.set_aspect('equal')

    # Etiquetas de filas y columnas
    ax.set_xticks([c + 0.5 for c in range(cols)])
    ax.set_xticklabels([str(c) for c in range(cols)], fontsize=8)
    ax.set_yticks([(rows - 1 - r) + 0.5 for r in range(rows)])
    ax.set_yticklabels([str(r) for r in range(rows)], fontsize=8)

    ax.set_xlabel('Columnas', fontsize=11, fontweight='bold', labelpad=8)
    ax.set_ylabel('Filas', fontsize=11, fontweight='bold', labelpad=8)

    # Título y detalles
    nombre_archivo = Path(json_path).name
    plt.title(f'Mapa Micromouse: {nombre_archivo}\nDimensiones: {rows}×{cols} | Casilla: {cell_cm} cm | Inicio: [{start_r}, {start_c}]',
              fontsize=12, fontweight='bold', pad=14)

    if dibujar_ruta and path:
        ax.legend(loc='upper right', framealpha=0.9, fontsize=9)

    plt.tight_layout()

    if salida_png:
        plt.savefig(salida_png, dpi=200, bbox_inches='tight')
        print(f'Imagen del laberinto guardada en: {salida_png}')

    if mostrar:
        plt.show()

    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Convierte el JSON de un laberinto en una imagen gráfica.')
    parser.add_argument('json', nargs='?', default='micromouse-centro.json', help='Ruta al archivo JSON del laberinto')
    parser.add_argument('-o', '--salida', default='laberinto.png', help='Nombre de archivo PNG de salida')
    parser.add_argument('--sin-ruta', action='store_true', help='No dibujar la ruta óptima')
    parser.add_argument('--mostrar', action='store_true', help='Mostrar la ventana interactiva de la gráfica')
    args = parser.parse_args()

    graficar_laberinto(
        json_path=args.json,
        salida_png=args.salida,
        mostrar=args.mostrar,
        dibujar_ruta=(not args.sin_ruta)
    )


if __name__ == '__main__':
    main()
