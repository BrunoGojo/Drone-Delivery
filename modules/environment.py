import pybullet as p
import pybullet_data
import random

class Environment:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        self.targets = [] # Lista de dicionários {'id': int, 'pos': [x,y,z], 'detected': bool}

    def setup_random_targets(self, count=5, area_size=10):
        """Cria cubos vermelhos aleatórios no mapa"""
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[1, 0, 0, 1])
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])

        for _ in range(count):
            x = random.uniform(-area_size, area_size)
            y = random.uniform(-area_size, area_size)
            pos = [x, y, 0.5]
            
            body_id = p.createMultiBody(baseMass=0, 
                                        baseCollisionShapeIndex=collision_shape, 
                                        baseVisualShapeIndex=visual_shape, 
                                        basePosition=pos)
            
            self.targets.append({'id': body_id, 'pos': pos, 'detected': False, 'delivered': False})
    
    def get_undiscovered_targets(self):
        # Retorna alvos que ainda não foram marcados como detectados
        return [t for t in self.targets if not t['detected']]

    def mark_detected(self, target_id):
        for t in self.targets:
            if t['id'] == target_id:
                t['detected'] = True
                # Muda cor para Amarelo (Detectado, na fila)
                p.changeVisualShape(t['id'], -1, rgbaColor=[1, 1, 0, 1])

    def mark_delivered(self, pos_coords):
        # Encontra o alvo pela posição (aproximada) e marca como entregue
        for t in self.targets:
            # Compara distancia simples para achar qual alvo é esse
            if t['detected'] and not t['delivered']:
                dist = sum([(a - b)**2 for a, b in zip(t['pos'], pos_coords)])
                if dist < 0.1:
                    t['delivered'] = True
                    # Muda cor para Verde (Entregue)
                    p.changeVisualShape(t['id'], -1, rgbaColor=[0, 1, 0, 1])
                    return