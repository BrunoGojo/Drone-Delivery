import pybullet as p
import pybullet_data
import random
import math
import config # Importa para pegar o raio de detecção

class Environment:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        self.targets = [] 

    def get_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def setup_smart_targets(self, count=50, area_size=20):
        """
        Gera pontos respeitando a regra de não aglomeração.
        - count: Total de pontos (50)
        - area_size: Tamanho do mapa (aumentado para caber 50 pontos espalhados)
        """
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[1, 0, 0, 1])
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])

        created_count = 0
        max_attempts = 5000 # Evita travar se não tiver espaço
        attempts = 0

        while created_count < count and attempts < max_attempts:
            attempts += 1
            
            # 1. Gera candidato aleatório
            x = random.uniform(-area_size, area_size)
            y = random.uniform(-area_size, area_size)
            candidate_pos = [x, y, 0.5]

            # 2. Verifica vizinhos próximos (Regra da Aglomeração)
            neighbors_in_range = 0
            too_close = False

            for t in self.targets:
                dist = self.get_distance(candidate_pos, t['pos'])
                
                # Regra A: Distância mínima física (para não sobrepor caixas)
                if dist < 1.0: 
                    too_close = True
                    break
                
                # Regra B: Contagem de vizinhos dentro do raio de detecção
                if dist <= config.DETECTION_RADIUS:
                    neighbors_in_range += 1

            # SE tiver muito perto de alguém OU se já tiver 2 vizinhos nessa área (Totalizaria 3 com ele)
            # A sua regra: "Não pode haver 3 pontos detectados juntos" (Total 4)
            # Vamos ser conservadores: Se já tem 2 vizinhos, não coloca o terceiro.
            if too_close or neighbors_in_range >= 2:
                continue # Pula essa tentativa e tenta outro lugar

            # 3. Se passou nos testes, cria o objeto
            body_id = p.createMultiBody(baseMass=0, 
                                        baseCollisionShapeIndex=collision_shape, 
                                        baseVisualShapeIndex=visual_shape, 
                                        basePosition=candidate_pos)
            
            self.targets.append({'id': body_id, 'pos': candidate_pos, 'detected': False, 'delivered': False})
            created_count += 1
            attempts = 0 # Reseta tentativas para o próximo ponto

        print(f"[AMBIENTE] Gerados {created_count} alvos de forma inteligente.")
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