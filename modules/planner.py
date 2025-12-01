import math
import itertools
import config

def calculate_distance(pos_a, pos_b):
    return math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

def generate_patrol_grid(area_size, height, step=4):
    """
    Gera uma lista de pontos em Zigue-Zague para cobrir a área.
    Isso garante que o drone 'escaneie' o mapa todo.
    """
    waypoints = []
    # Cria uma grade de -area a +area
    x_range = range(int(-area_size), int(area_size), step)
    y_range = range(int(-area_size), int(area_size), step)
    
    flip = False
    for x in x_range:
        # Se flip é True, inverte a ordem do Y para fazer o movimento de cobra
        current_y_range = reversed(y_range) if flip else y_range
        for y in current_y_range:
            waypoints.append([x, y, height])
        flip = not flip
        
    return waypoints

def optimize_route(current_pos, pending_targets, base_pos):
    """
    Recebe: Posição Atual, Lista de Alvos Pendentes, Posição da Base.
    Retorna: Lista de alvos reordenada para o menor caminho total.
    """
    if not pending_targets:
        return []

    # Se tiver muitos pontos (>8), permutações ficam lentas. 
    # Usamos Vizinho Mais Próximo (Nearest Neighbor) se a lista for grande.
    if len(pending_targets) > 8:
        path = []
        curr = current_pos
        unvisited = pending_targets[:]
        while unvisited:
            # Pega o mais próximo do ponto atual
            nearest = min(unvisited, key=lambda x: calculate_distance(curr, x))
            path.append(nearest)
            unvisited.remove(nearest)
            curr = nearest
        return path

    # Algoritmo exato (Força Bruta) para poucos pontos
    best_route = []
    min_total_dist = float('inf')

    for perm in itertools.permutations(pending_targets):
        current_dist = 0
        last_pos = current_pos
        for target in perm:
            current_dist += calculate_distance(last_pos, target)
            last_pos = target
        current_dist += calculate_distance(last_pos, base_pos)

        if current_dist < min_total_dist:
            min_total_dist = current_dist
            best_route = list(perm)

    print(f"[PLANNER] Rota de entrega otimizada. Custo: {min_total_dist:.2f}")
    return best_route