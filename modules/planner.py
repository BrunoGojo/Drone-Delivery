import math
import itertools
import config

def calculate_distance(pos_a, pos_b):
    # Distância Euclidiana simples (ignorando altura Z para simplificar rota 2D)
    return math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

def optimize_route(current_pos, pending_targets, base_pos):
    """
    Recebe: Posição Atual, Lista de Alvos Pendentes, Posição da Base.
    Retorna: Lista de alvos reordenada para o menor caminho total.
    """
    if not pending_targets:
        return []

    # Se houver poucos pontos (< 8), usamos força bruta (permutação) para a rota perfeita.
    # Se houver muitos, seria melhor usar 'Vizinho Mais Próximo'.
    
    best_route = []
    min_total_dist = float('inf')

    # Testa todas as ordens possíveis de visita
    for perm in itertools.permutations(pending_targets):
        current_dist = 0
        last_pos = current_pos
        
        # Calcula custo: Atual -> P1 -> P2 ... -> Pn
        for target in perm:
            # target é um tuple/list (x, y, z)
            current_dist += calculate_distance(last_pos, target)
            last_pos = target
        
        # Soma a volta para a base: Pn -> Base
        current_dist += calculate_distance(last_pos, base_pos)

        if current_dist < min_total_dist:
            min_total_dist = current_dist
            best_route = list(perm)

    print(f"[PLANNER] Rota Recalculada. Custo total estimado: {min_total_dist:.2f}")
    return best_route