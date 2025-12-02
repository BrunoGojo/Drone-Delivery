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
        Gera esferas com regra estrita anti-aglomeração.
        """
        # Esferas vermelhas, raio 0.15
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.15, rgbaColor=[1, 0, 0, 1])
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.15)

        created_count = 0
        max_attempts = 20000  # Aumentei tentativas pois vai ser difícil achar espaço
        attempts = 0

        while created_count < count and attempts < max_attempts:
            attempts += 1
            
            x = random.uniform(-area_size, area_size)
            y = random.uniform(-area_size, area_size)
            candidate_pos = [x, y, 0.15]

            neighbors_in_range = 0
            too_close = False

            for t in self.targets:
                dist = self.get_distance(candidate_pos, t['pos'])
                
                # MUDANÇA 1: Aumentei o isolamento físico de 0.35 para 0.8
                # Isso impede que as esferas fiquem "coladas", espalhando mais o grupo.
                if dist < 0.8: 
                    too_close = True
                    break
                
                # MUDANÇA 2: Margem de Segurança do Sensor
                # Verificamos um raio um pouco MAIOR (x1.2) que o do drone.
                # Se nesse raio ampliado já tiver gente, a gente rejeita.
                if dist <= (config.DETECTION_RADIUS * 1.2):
                    neighbors_in_range += 1

            # MUDANÇA 3: Limite Rígido Estístico
            # Rejeitamos se já houver 2 ou mais vizinhos.
            # Objetivo: Tentar criar grupos de 1 ou 2. 
            # Como a área é densa, os "grupos de 2" vão acabar encostando em "solitários",
            # formando os trios (3) que você quer, mas dificultando formar 4 ou 5.
            if too_close or neighbors_in_range >= 2:
                continue 

            body_id = p.createMultiBody(baseMass=0, 
                                        baseCollisionShapeIndex=collision_shape, 
                                        baseVisualShapeIndex=visual_shape, 
                                        basePosition=candidate_pos)
            
            self.targets.append({'id': body_id, 'pos': candidate_pos, 'detected': False, 'delivered': False})
            created_count += 1
            attempts = 0 

        print(f"[AMBIENTE] Geradas {created_count} esferas com restrição rigorosa.")
        if created_count < count:
            print(f"[AVISO] Só foi possível gerar {created_count} pontos. A área é muito pequena para essa regra.")

    def mark_detected(self, target_id):
        for t in self.targets:
            if t['id'] == target_id:
                t['detected'] = True
                p.changeVisualShape(t['id'], -1, rgbaColor=[1, 1, 0, 1])

    def mark_delivered(self, pos_coords):
        for t in self.targets:
            if t['detected'] and not t['delivered']:
                # Verifica proximidade para confirmar entrega
                dist = sum([(a - b)**2 for a, b in zip(t['pos'], pos_coords)])
                if dist < 0.5: # Tolerância
                    t['delivered'] = True
                    p.changeVisualShape(t['id'], -1, rgbaColor=[0, 1, 0, 1])
                    return