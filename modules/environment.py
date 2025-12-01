import pybullet as p
import pybullet_data
import random
import math
import config

class Environment:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        self.targets = [] 

    def get_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def setup_smart_targets(self, count=50, area_size=20):
        """
        Gera esferas (targets) respeitando a regra de não aglomeração.
        """
        # MUDANÇA AQUI: Trocamos GEOM_BOX por GEOM_SPHERE
        # radius=0.15 deixa elas menores e mais fáceis de desviar
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[1, 0, 0, 1])
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.15)

        created_count = 0
        max_attempts = 5000 
        attempts = 0

        while created_count < count and attempts < max_attempts:
            attempts += 1
            
            x = random.uniform(-area_size, area_size)
            y = random.uniform(-area_size, area_size)
            # z=0.15 para a esfera ficar 'apoiada' no chão, e não flutuando muito alto
            candidate_pos = [x, y, 0.15]

            neighbors_in_range = 0
            too_close = False

            for t in self.targets:
                dist = self.get_distance(candidate_pos, t['pos'])
                
                # Regra A: Distância mínima física (menor agora que são esferas pequenas)
                if dist < 0.35: 
                    too_close = True
                    break
                
                # Regra B: Contagem de vizinhos
                if dist <= config.DETECTION_RADIUS:
                    neighbors_in_range += 1

            if too_close or neighbors_in_range >= 2:
                continue 

            body_id = p.createMultiBody(baseMass=0, 
                                        baseCollisionShapeIndex=collision_shape, 
                                        baseVisualShapeIndex=visual_shape, 
                                        basePosition=candidate_pos)
            
            self.targets.append({'id': body_id, 'pos': candidate_pos, 'detected': False, 'delivered': False})
            created_count += 1
            attempts = 0 

        print(f"[AMBIENTE] Gerados {created_count} esferas de entrega.")
        if created_count < count:
            print("[AVISO] Área muito pequena para essa quantidade de restrições.")

    def mark_detected(self, target_id):
        for t in self.targets:
            if t['id'] == target_id:
                t['detected'] = True
                p.changeVisualShape(t['id'], -1, rgbaColor=[1, 1, 0, 1])

    def mark_delivered(self, pos_coords):
        for t in self.targets:
            if t['detected'] and not t['delivered']:
                dist = sum([(a - b)**2 for a, b in zip(t['pos'], pos_coords)])
                if dist < 0.1:
                    t['delivered'] = True
                    p.changeVisualShape(t['id'], -1, rgbaColor=[0, 1, 0, 1])
                    return