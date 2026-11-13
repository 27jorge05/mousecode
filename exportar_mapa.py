"""Convierte un JSON de MouseLab al vector de paredes del firmware, sin dependencias."""
from pathlib import Path
import argparse
import hashlib
import json

HERE = Path(__file__).resolve().parent


def convert(path):
    raw = path.read_bytes()
    maze = json.loads(raw)
    rows, cols = maze['rows'], maze['cols']
    if type(rows) is not int or type(cols) is not int or rows < 2 or cols < 2 or rows * cols > 256:
        raise ValueError('El navegador de este ESP32 admite de 2×2 hasta 256 casillas en total.')
    if maze.get('goalMode') != 'center':
        raise ValueError('Este firmware requiere meta central.')
    walls = maze['walls']
    if len(walls) != rows or any(len(row) != cols for row in walls):
        raise ValueError('Dimensiones del vector de paredes incorrectas.')
    directions = [('top', -1, 0, 'bottom'), ('right', 0, 1, 'left'),
                  ('bottom', 1, 0, 'top'), ('left', 0, -1, 'right')]
    vector = []
    for r in range(rows):
        values = []
        for c in range(cols):
            mask = 0
            for d, (key, dr, dc, opposite) in enumerate(directions):
                value = walls[r][c][key]
                if type(value) is not bool:
                    raise ValueError('Cada pared debe ser true o false.')
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    if not value:
                        raise ValueError('El borde exterior debe estar cerrado.')
                elif value != walls[nr][nc][opposite]:
                    raise ValueError('Hay paredes asimétricas entre casillas vecinas.')
                if value:
                    mask |= 1 << d
            values.append(mask)
        vector.append(values)
    start = maze['start']
    sr, sc = start['r'], start['c']
    if type(sr) is not int or type(sc) is not int or not (0 <= sr < rows and 0 <= sc < cols):
        raise ValueError('Inicio fuera del mapa.')
    openings = [d for d in range(4) if not (vector[sr][sc] & (1 << d))]
    if len(openings) != 1:
        raise ValueError('La casilla inicial debe tener una sola salida: orientación inequívoca.')
    cell = maze.get('geometry', {}).get('cellCm', 18)
    if type(cell) not in (int, float) or not (2 <= cell <= 100):
        raise ValueError('Medida de casilla inválida.')
    goals = {(r, c) for r in {(rows - 1) // 2, rows // 2} for c in {(cols - 1) // 2, cols // 2}}
    queue, seen = [(sr, sc)], {(sr, sc)}
    for r, c in queue:
        for d, (_, dr, dc, _) in enumerate(directions):
            if not vector[r][c] & (1 << d) and (r+dr, c+dc) not in seen:
                seen.add((r+dr, c+dc))
                queue.append((r+dr, c+dc))
    if not seen & goals:
        raise ValueError('El centro no es accesible desde el inicio.')
    digest = hashlib.sha256(raw).hexdigest()
    lines = ['#pragma once', '#include <stdint.h>', '',
             '// DETECCION HARDCODEADA: mapa del simulador, NO sensores IR.',
             '// Editable como vector. Indice = fila * COLUMNAS + columna.',
             '// Bits de pared: norte=1, este=2, sur=4, oeste=8. 0=abierto, 15=cerrado.',
             '// Regenerar con: python3 exportar_mapa.py micromouse-centro.json',
             'namespace MapaSimulador {', f'constexpr int FILAS = {rows};', f'constexpr int COLUMNAS = {cols};',
             f'constexpr int INICIO_FILA = {sr};', f'constexpr int INICIO_COLUMNA = {sc};',
             f'constexpr int INICIO_RUMBO = {openings[0]}; // norte=0, este=1, sur=2, oeste=3',
             f'constexpr float CASILLA_CM = {float(cell):.6f}f;',
             f'constexpr char ARCHIVO[] = {json.dumps(path.name, ensure_ascii=False)};', f'constexpr char SHA256[] = "{digest}";',
             'constexpr uint8_t PAREDES[] = {']
    lines += ['    ' + ', '.join(f'{v:2}' for v in row) + f', // fila {r}' for r, row in enumerate(vector)]
    lines += ['};', 'static_assert(sizeof(PAREDES) / sizeof(PAREDES[0]) == FILAS * COLUMNAS, "Revisar longitud del vector de paredes");',
              'static_assert(FILAS * COLUMNAS <= 256, "Capacidad maxima: 256 casillas");', '} // namespace MapaSimulador', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapa', nargs='?', type=Path, default=HERE / 'micromouse-centro.json')
    parser.add_argument('--salida', type=Path, default=HERE / 'src/mapa/MapaSimulador.h')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    try:
        content = convert(args.mapa)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    if args.check:
        if not args.salida.exists() or args.salida.read_text() != content:
            parser.error('MapaSimulador.h no coincide con el JSON. Regenera el vector o revisa los cambios manuales.')
        print('Vector sincronizado con el JSON y mapa válido.')
    else:
        args.salida.write_text(content)
        print(f'Mapa generado: {args.salida}')


if __name__ == '__main__':
    main()
