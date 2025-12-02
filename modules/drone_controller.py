import pybullet as p
import math
import config

class PIDController:
    """Implementação clássica de PID para um eixo"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def compute(self, target, current, dt):
        error = target - current
        
        # Integral com limite (Anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, 2.0), -2.0)
        
        # Derivativo
        derivative = (error - self.prev_error) / dt
        
        # Saída
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        return output

class Drone:
    def __init__(self, start_pos):
        # Drone visual (cubo azul)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0, 0, 1, 1])
        # Aumentei o 'linearDamping' para o ar "segurar" um pouco o drone naturalmente
        self.body_id = p.createMultiBody(baseMass=1, 
                                         baseCollisionShapeIndex=col, 
                                         baseVisualShapeIndex=vis, 
                                         basePosition=start_pos)
        
        self.current_target = None
        self.route_queue = [] 
        
        # --- SISTEMA PID 3 EIXOS ---
        self.pid_x = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])
        self.pid_y = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])
        self.pid_z = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])
    
    def get_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return list(pos)

    def scan_for_targets(self, env_targets):
        my_pos = self.get_position()
        found_new = []
        for t in env_targets:
            if not t['detected']:
                dist = math.sqrt((my_pos[0] - t['pos'][0])**2 + (my_pos[1] - t['pos'][1])**2)
                if dist <= config.DETECTION_RADIUS:
                    found_new.append(t)
        return found_new

    def move_step(self):
        """
        Executa um passo de controle usando PID + Estabilização Angular.
        """
        # --- ESTABILIZADOR (GIROSCÓPIO VIRTUAL) ---
        # Isso impede que o drone gire descontroladamente.
        # Forçamos a orientação a ficar "em pé" (Quaternion [0,0,0,1])
        current_pos, _ = p.getBasePositionAndOrientation(self.body_id)
        p.resetBasePositionAndOrientation(self.body_id, current_pos, [0, 0, 0, 1])

        if not self.current_target:
            # Para o drone completamente
            p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
            return False 

        target = self.current_target
        
        # Verifica chegada
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        dz = target[2] - current_pos[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist < config.ARRIVAL_THRESHOLD:
            # Reseta PID ao chegar para evitar solavanco na saída
            self.pid_x.prev_error = 0; self.pid_y.prev_error = 0; self.pid_z.prev_error = 0
            self.pid_x.integral = 0;   self.pid_y.integral = 0;   self.pid_z.integral = 0
            return True 

        # --- CÁLCULO DO PID ---
        dt = config.TIME_STEP
        vx = self.pid_x.compute(target[0], current_pos[0], dt)
        vy = self.pid_y.compute(target[1], current_pos[1], dt)
        vz = self.pid_z.compute(target[2], current_pos[2], dt)
        
        # Limitador de Velocidade (Safety Clamp)
        limit = config.FLIGHT_SPEED
        vx = max(min(vx, limit), -limit)
        vy = max(min(vy, limit), -limit)
        vz = max(min(vz, limit), -limit)

        # Aplica Velocidade Linear (PID) e Zera Velocidade Angular (Travamento)
        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, vz], angularVelocity=[0, 0, 0])
        return False