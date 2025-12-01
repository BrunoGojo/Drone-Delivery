import pybullet as p
import math
import config

class Drone:
    def __init__(self, start_pos):
        # Carrega um modelo simples de drone (ex: pato ou cubo se não tiver o urdf ainda)
        # Para usar o cubo como drone temporário:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0, 0, 1, 1])
        self.body_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=start_pos)
        
        self.current_target = None
        self.route_queue = [] # Lista de posições
    
    def get_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return list(pos)

    def scan_for_targets(self, env_targets):
        """
        Simula o sensor. Verifica se algum alvo 'não detectado' está dentro do raio.
        Retorna: Lista de novos alvos encontrados (posições).
        """
        my_pos = self.get_position()
        found_new = []

        for t in env_targets:
            if not t['detected']:
                # Calcula distancia
                dist = math.sqrt((my_pos[0] - t['pos'][0])**2 + (my_pos[1] - t['pos'][1])**2)
                if dist <= config.DETECTION_RADIUS:
                    found_new.append(t)
        return found_new

    def move_step(self):
        """Move o drone em direção ao alvo atual usando velocidade linear"""
        if not self.current_target:
            # Se não tem alvo, para
            p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0])
            return False # Não chegou

        my_pos = self.get_position()
        target = self.current_target

        # Vetor direção
        dx = target[0] - my_pos[0]
        dy = target[1] - my_pos[1]
        dz = target[2] - my_pos[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist < config.ARRIVAL_THRESHOLD:
            return True # Chegou no alvo

        # Normaliza e aplica velocidade
        vx = (dx / dist) * config.FLIGHT_SPEED
        vy = (dy / dist) * config.FLIGHT_SPEED
        vz = (dz / dist) * config.FLIGHT_SPEED

        # Simples controle de velocidade (sem inércia complexa para focar na rota)
        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, vz])
        return False