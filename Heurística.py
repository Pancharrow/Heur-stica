# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 19:24:09 2025

@author: Francisco FV C
"""
# --------------------------------------------------------------------------------
# Importaciones
# --------------------------------------------------------------------------------
import math
import heapq

# --------------------------------------------------------------------------------
# Datos del grafo
# --------------------------------------------------------------------------------
grafo = {
    # --- norte_camellon ---
    (5, 6): 120, (6, 5): 120,
    (6, 8): 60,  (8, 6): 60,
    
    # --- norte_dobles ---
    (1, 2): 50,   (2, 1): 50,
    (2, 3): 160,  (3, 2): 160,
    (2, 5): 80,   (5, 2): 80,
    (3, 4): 40,   (4, 3): 40,
    (3, 6): 50,   (6, 3): 50,
    (7, 8): 60,   (8, 7): 60,
    (8, 10): 50,  (10, 8): 50,
    (9, 10): 90,  (10, 9): 90,
    (10, 11): 30, (11, 10): 30,
    (11, 12): 60, (12, 11): 60,
    (11, 19): 120, (19, 11): 120,
    (19, 20): 90, (20, 19): 90,
    (5, 20): 170, (20, 5): 170,
    (5, 13): 300, (13, 5): 300,
    (20, 21): 60, (21, 20): 60,
    (13, 14): 50, (14, 13): 50,
    (13, 15): 30, (15, 13): 30,
    (20, 22): 100, (22, 20): 100,
    (15, 22): 190, (22, 15): 190,
    (15, 18): 70,  (18, 15): 70,
    (17, 18): 50,  (18, 17): 50,
    (17, 16): 40,  (16, 17): 40,
    (18, 23): 110, (23, 18): 110,
    (17, 23): 160, (23, 17): 160,
    (19, 25): 340, (25, 19): 340,
    (24, 25): 30,  (25, 24): 30,
    (25, 26): 190, (26, 25): 190,
    (26, 27): 130, (27, 26): 130,
    (26, 28): 60,  (28, 26): 60,
    (22, 28): 390, (28, 22): 390,
    (23, 37): 560, (37, 23): 560,
    (28, 29): 100, (29, 28): 100,
    (29, 30): 70,  (30, 29): 70,
    (30, 31): 50,  (31, 30): 50,
    (30, 32): 70,  (32, 30): 70,
    (32, 34): 50,  (34, 32): 50,
    (32, 33): 60,  (33, 32): 60,
    (29, 35): 30,  (35, 29): 30,
    (35, 36): 90,  (36, 35): 90,
    (35, 37): 130, (37, 35): 130,
    (37, 38): 90,  (38, 37): 90,
    (37, 40): 280, (40, 37): 280,
    (38, 40): 350, (40, 38): 350,
    (38, 39): 240, (39, 38): 240,
    (40, 41): 200, (41, 40): 200,
    
    # --- centro_norte_camellon ---
    (48, 49): 170, (49, 48): 170,
    
    # --- centro_norte_dobles ---
    (42, 41): 180, (41, 42): 180,
    (41, 43): 170, (43, 41): 170,
    (43, 44): 40,  (44, 43): 40,
    (43, 47): 320, (47, 43): 320,
    (44, 45): 170, (45, 44): 170,
    (45, 48): 330, (48, 45): 330,
    (49, 50): 70,  (50, 49): 70,
    (69, 51): 610, (51, 69): 610,
    (46, 51): 250, (51, 46): 250,
    (45, 52): 190, (52, 45): 190,
    (52, 53): 180, (53, 52): 180,
    (53, 54): 90,  (54, 53): 90,
    (53, 55): 40,  (55, 53): 40,
    (55, 56): 80,  (56, 55): 80,
    (55, 57): 30,  (57, 55): 30,
    (57, 58): 100, (58, 57): 100,
    (57, 59): 180, (59, 57): 180,
    (59, 60): 40,  (60, 59): 40,
    (60, 52): 270, (52, 60): 270,
    (60, 61): 90,  (61, 60): 90,
    (61, 62): 60,  (62, 61): 60,
    (61, 65): 80,  (65, 61): 80,
    (62, 64): 80,  (64, 62): 80,
    (62, 63): 40,  (63, 62): 40,
    (59, 67): 30,  (67, 59): 30,
    (59, 66): 30,  (66, 59): 30,
    (66, 69): 50,  (69, 66): 50,
    (67, 68): 20,  (68, 67): 20,
    (68, 69): 70,  (69, 68): 70,
    (67, 71): 160, (71, 67): 160,
    (68, 70): 110, (70, 68): 110,
    (66, 46): 730, (46, 66): 730,
    
    # --- centro_sur_dobles ---
    (72, 73): 340, (73, 72): 340,
    (44, 75): 430, (75, 44): 430,
    (75, 74): 70,  (74, 75): 70,
    (75, 76): 50,  (76, 75): 50,
    (76, 77): 70,  (77, 76): 70,
    (77, 78): 60,  (78, 77): 60,
    (77, 79): 110, (79, 77): 110,
    (76, 80): 150, (80, 76): 150,
    (80, 83): 170, (83, 80): 170,
    (80, 81): 70,  (81, 80): 70,
    (81, 82): 260, (82, 81): 260,
    (81, 84): 240, (84, 81): 240,
    (84, 85): 130, (85, 84): 130,
    (84, 86): 110, (86, 84): 110,
    (86, 87): 200, (87, 86): 200,
    (86, 88): 100, (88, 86): 100,
    (88, 91): 240, (91, 88): 240,
    (88, 89): 40,  (89, 88): 40,
    (89, 92): 230, (92, 89): 230,
    (91, 92): 40,  (92, 91): 40,
    (91, 94): 220, (94, 91): 220,
    (92, 93): 130, (93, 92): 130,
    (89, 90): 120, (90, 89): 120,
    (90, 90): 540,  # bucle a sí mismo
    (88, 46): 810, (46, 88): 810,
    (89, 51): 840, (51, 89): 840,
    
    # --- sur_dobles ---
    (95, 96): 100, (96, 95): 100,
    (96, 97): 50,  (97, 96): 50,
    (96, 98): 200, (98, 96): 200,
    (98, 99): 130, (99, 98): 130,
    (99, 94): 200, (94, 99): 200,
    (94, 100): 60, (100, 94): 60,
    (100, 93): 80, (93, 100): 80,
    (100, 101): 130, (101, 100): 130,
    (99, 102): 160, (102, 99): 160,
    (101, 102): 150, (102, 101): 150,
    (101, 106): 30,  (106, 101): 30,
    (102, 103): 120, (103, 102): 120,
    (103, 104): 160, (104, 103): 160,
    (105, 106): 340, (106, 105): 340,
    (106, 107): 110, (107, 106): 110,
    (107, 108): 110, (108, 107): 110,
    (107, 110): 410, (110, 107): 410,
    (110, 109): 90,  (109, 110): 90,
    (110, 111): 190, (111, 110): 190,
    (99, 112): 1860, (112, 99): 1860,
    (103, 112): 1400, (112, 103): 1400,
    
    # --- depot_sencillos ---
    (0, 112): 3870, (112, 0): 3870,
    (112, 98): 2020, (98, 112): 2020,
    (98, 82): 900,   (82, 98): 900,
    (82, 72): 150,   (72, 82): 150,
    (42, 0): 1200,   (0, 42): 1200,
    (72, 42): 170,   (42, 72): 170,
    (0, 24): 1600,   (24, 0): 1600,
    (42, 24): 710,   (24, 42): 710,
    
    # --- depot_dobles ---
    (0, 98): 1770,   (98, 0): 1770,
}

# --------------------------------------------------------------------------------
# 1) Función para calcular el costo de una ruta
# --------------------------------------------------------------------------------
def costo_ruta(tour):
    total = 0
    for i in range(len(tour) - 1):
        u = tour[i]
        v = tour[i+1]
        total += grafo[(u, v)]
    return total

# --------------------------------------------------------------------------------
# 2) Función para detectar subciclos en una ruta
# --------------------------------------------------------------------------------
def ciclos(ruta):
    lista_ciclos = []
    n = len(ruta)
    for inicio in range(n - 1):
        for fin in range(inicio + 1, n):
            if ruta[inicio] == ruta[fin]:
                sub_ciclo = ruta[inicio:fin + 1]
                # Requisitos: >= 3 nodos y no sea el tour entero
                if len(sub_ciclo) > 2 and not (inicio == 0 and fin == n - 1):
                    lista_ciclos.append(sub_ciclo)
    return lista_ciclos

# --------------------------------------------------------------------------------
# 3) Función para quitar un subciclo de una ruta
# --------------------------------------------------------------------------------
def quitar_sub_ciclo(ruta, ciclo):
    copia_ruta = ruta[:]
    str_ruta = ",".join(map(str, copia_ruta))
    str_ciclo = ",".join(map(str, ciclo))

    indice = str_ruta.find(str_ciclo)
    if indice == -1:
        return copia_ruta

    # Reemplazamos la primera ocurrencia
    ruta_sin_ciclo = str_ruta.replace(str_ciclo, "", 1)
    nodos = [p for p in ruta_sin_ciclo.split(",") if p != ""]
    try:
        ruta_final = list(map(int, nodos))
    except ValueError:
        return copia_ruta

    # Insertar último nodo del subciclo en la posición adecuada
    ultimo_nodo = ciclo[-1]
    partes_antes_de_ciclo = [p for p in str_ruta[:indice].split(",") if p]
    lugar_insertar = len(partes_antes_de_ciclo)
    if lugar_insertar > len(ruta_final):
        lugar_insertar = len(ruta_final)

    ruta_final.insert(lugar_insertar, ultimo_nodo)
    return ruta_final

# --------------------------------------------------------------------------------
# 4) Dijkstra para distancias mínimas
# --------------------------------------------------------------------------------
def dijkstra(grafo, inicio, fin):
    """
    Calcula la distancia mínima desde 'inicio' hasta 'fin'.
    Devuelve (distancia, ruta) usando un heap de prioridad.
    """
    nodos = set()
    for (u, v) in grafo:
        nodos.add(u)
        nodos.add(v)
    
    distancia = {n: float('inf') for n in nodos}
    predecesor = {n: None for n in nodos}
    distancia[inicio] = 0
    pq = []
    heapq.heappush(pq, (0, inicio))

    while pq:
        dist_actual, nodo_actual = heapq.heappop(pq)
        if nodo_actual == fin:
            break

        for (u, v), peso in grafo.items():
            if u == nodo_actual:
                nueva_dist = dist_actual + peso
                if nueva_dist < distancia[v]:
                    distancia[v] = nueva_dist
                    predecesor[v] = u
                    heapq.heappush(pq, (nueva_dist, v))

    # Reconstruir la ruta
    ruta = []
    nodo = fin
    while nodo is not None:
        ruta.append(nodo)
        nodo = predecesor[nodo]
    ruta.reverse()

    return distancia[fin], ruta

# --------------------------------------------------------------------------------
# 5) Añadir un subciclo a una ruta
# --------------------------------------------------------------------------------
def anadir_subciclo(ruta, subciclo, grafo):
    """
    Inserta un subciclo en la ruta manteniendo coherencia,
    usando Dijkstra para entrar y salir del subciclo.
    """
    if not subciclo:
        return ruta[:]

    X = subciclo[0]

    # Buscar el nodo más cercano en la ruta a X
    mejor_nodo_entrada = None
    mejor_dist_entrada = float('inf')
    mejor_ruta_entrada = None

    for nodo in ruta:
        dist, ruta_entrada = dijkstra(grafo, nodo, X)
        if dist < mejor_dist_entrada:
            mejor_dist_entrada = dist
            mejor_nodo_entrada = nodo
            mejor_ruta_entrada = ruta_entrada

    if mejor_nodo_entrada is None or mejor_ruta_entrada is None:
        return ruta

    idx = ruta.index(mejor_nodo_entrada)
    nodos_a_insertar = subciclo[:]
    ultima_parada = nodos_a_insertar[-1]

    # Camino de regreso usando Dijkstra
    mejor_nodo_salida = mejor_nodo_entrada
    _, camino_de_vuelta = dijkstra(grafo, ultima_parada, mejor_nodo_salida)
    if camino_de_vuelta is None:
        return ruta

    # Evitar duplicaciones
    if mejor_ruta_entrada[-1] == X:
        mejor_ruta_entrada = mejor_ruta_entrada[:-1]
    if camino_de_vuelta[0] == ultima_parada:
        camino_de_vuelta = camino_de_vuelta[1:]

    nueva_ruta = (
        ruta[:idx]
        + mejor_ruta_entrada
        + nodos_a_insertar
        + camino_de_vuelta
        + ruta[idx+1:]
    )
    return nueva_ruta

# --------------------------------------------------------------------------------
# 6) Generar vecinos con cycle_trade
# --------------------------------------------------------------------------------
def cycle_trade(T):
    vecinos = []

    # Localizar la ruta más "cara" y la más "barata"
    idx_caro = max(range(len(T)), key=lambda i: costo_ruta(T[i]))
    idx_barato = min(range(len(T)), key=lambda i: costo_ruta(T[i]))

    t1 = T[idx_caro]
    t2 = T[idx_barato]
    subciclos = ciclos(t1)

    for c in subciclos:
        t1_nuevo = quitar_sub_ciclo(t1, c)
        t2_nuevo = anadir_subciclo(t2, c, grafo)

        # Construimos la nueva solución
        T_vecino = []
        for i, tour in enumerate(T):
            if i != idx_caro and i != idx_barato:
                T_vecino.append(tour)
        T_vecino.append(t1_nuevo)
        T_vecino.append(t2_nuevo)

        # Almacenamos la transición
        transicion = (tuple(c), idx_caro, idx_barato)
        vecinos.append((T_vecino, transicion))

    return vecinos

# --------------------------------------------------------------------------------
# 7) Función objetivo para Weighted k-CPP
# --------------------------------------------------------------------------------
def funcion_objetivo(solucion, lambda_):
    """
    FO = lambda * max(costo_ruta(t)) + (1 - lambda) * sum(costo_ruta(t))
    """
    costos = [costo_ruta(t) for t in solucion]
    return lambda_ * max(costos) + (1 - lambda_) * sum(costos)

# --------------------------------------------------------------------------------
# 8) Búsqueda Tabú con cycle_trade
# --------------------------------------------------------------------------------
def tabu_search_cycle_trade(solucion_inicial, lambda_, 
                            max_iter, max_sin_mejora, 
                            max_tabu_size):
    mejor_sol = solucion_inicial
    mejor_fo = funcion_objetivo(mejor_sol, lambda_)

    sol_actual = solucion_inicial
    fo_actual = mejor_fo

    tabu_list = []
    sin_mejora = 0

    for it in range(max_iter):
        print(f"\n[Iter {it}] Sol. actual = {sol_actual}")
        print(f"[Iter {it}] FO actual = {fo_actual}, FO mejor = {mejor_fo}")

        vecinos = cycle_trade(sol_actual)
        if not vecinos:
            print(f"[Iter {it}] No hay vecinos, se detiene.")
            break

        mejor_vecino = None
        mejor_vecino_fo = math.inf
        mejor_vecino_transicion = None

        for (sol_vecina, transicion) in vecinos:
            # Checar si la transición está en tabu_list
            if transicion in tabu_list:
                # Criterio de aspiración
                fo_vecina = funcion_objetivo(sol_vecina, lambda_)
                if fo_vecina < mejor_fo:
                    if fo_vecina < mejor_vecino_fo:
                        mejor_vecino = sol_vecina
                        mejor_vecino_fo = fo_vecina
                        mejor_vecino_transicion = transicion
                else:
                    continue
            else:
                fo_vecina = funcion_objetivo(sol_vecina, lambda_)
                if fo_vecina < mejor_vecino_fo:
                    mejor_vecino = sol_vecina
                    mejor_vecino_fo = fo_vecina
                    mejor_vecino_transicion = transicion

        if mejor_vecino is None:
            print(f"[Iter {it}] No hay vecinos válidos, se detiene.")
            break

        # Actualizamos solución actual
        sol_actual = mejor_vecino
        fo_actual = mejor_vecino_fo

        # Agregamos la transición a la lista tabú
        if mejor_vecino_transicion:
            tabu_list.append(mejor_vecino_transicion)
            if len(tabu_list) > max_tabu_size:
                tabu_list.pop(0)

        if fo_actual < mejor_fo:
            mejor_sol = sol_actual
            mejor_fo = fo_actual
            sin_mejora = 0
        else:
            sin_mejora += 1

        if sin_mejora >= max_sin_mejora:
            print(f"[Iter {it}] Alcanzadas {max_sin_mejora} iteraciones sin mejora. Fin.")
            break

    return mejor_sol, mejor_fo

# --------------------------------------------------------------------------------
# 9) Ejemplo de uso
# --------------------------------------------------------------------------------
T_inicial = [
        [
            0, 24, 25, 26, 27, 26, 28, 22, 15, 13, 14, 13, 5, 6, 3, 4, 3, 2, 
            1, 2, 5, 13, 15, 22, 28, 29, 30, 32, 33, 32, 34, 32, 30, 31, 30, 
            29, 35, 36, 35, 37, 40, 41, 42, 0
        ],
        [
            0, 98, 96, 97, 96, 95, 96, 98, 99, 112, 103, 104, 103, 102, 99, 102,
            101, 106, 107, 110, 111, 110, 109, 110, 107, 108, 107, 106, 105, 106,
            101, 100, 93, 92, 91, 92, 89, 88, 86, 87, 86, 84, 85, 84, 86, 88, 46, 
            66, 59, 57, 58, 57, 55, 56, 55, 53, 54, 53, 52, 45, 48, 49, 50, 49, 
            48, 45, 44, 43, 47, 43, 41, 42, 0
        ],
        [
            0, 24, 25, 19, 20, 21, 20, 19, 11, 12, 11, 10, 9, 10, 8, 7, 8, 6, 8,
            6, 5, 20, 22, 15, 18, 23, 17, 16, 17, 18, 23, 37, 38, 39, 38, 40, 41,
            42, 0
        ],
        [
            0, 98, 99, 94, 100, 94, 91, 88, 89, 90, 90, 89, 51, 46, 51, 69, 66, 69, 
            68, 70, 68, 67, 71, 67, 59, 67, 59, 60, 61, 65, 61, 62, 63, 62, 64, 62, 
            61, 60, 52, 45, 44, 43, 44, 75, 74, 75, 76, 77, 79, 77, 78, 77, 76, 80, 
            83, 80, 81, 84, 81, 82, 72, 73, 72, 42, 0
        ]
    ]
    
print("\n=== Iniciando Tabu Search con cycle_trade ===\n")
mejor_sol, mejor_fo = tabu_search_cycle_trade(
    T_inicial,
    lambda_=1,
    max_iter=500,
    max_sin_mejora=10,
    max_tabu_size=10
)
    
print("\n=== RESULTADO FINAL ===")
for i, ruta in enumerate(mejor_sol, start=1):
    c = costo_ruta(ruta)
    print(f"  Ruta {i}: {ruta}  |  Costo = {c}")
print(f"Mejor Función Objetivo (FO) = {mejor_fo}")
