import pybullet as p
import math
import config
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        
    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        # Limita integral para evitar acumulo excessivo
        self.integral = max(min(self.integral, 2.0), -2.0)
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

class Drone:
    def __init__(self, start_pos):
        # Drone visual (cubo azul)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0, 0, 1, 1])
        self.body_id = p.createMultiBody(baseMass=1, 
                                         baseCollisionShapeIndex=col, 
                                         baseVisualShapeIndex=vis, 
                                         basePosition=start_pos)
        
        self.current_target = None
        self.route_queue = [] 
        
        # PIDs
        self.pid_x = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])
        self.pid_y = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])
        self.pid_z = PIDController(config.PID_GAINS['kp'], config.PID_GAINS['ki'], config.PID_GAINS['kd'])

    def get_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return list(pos)

    # --- ESTA É A FUNÇÃO QUE ESTAVA FALTANDO ---
    def check_crash(self):
        pos, rot = p.getBasePositionAndOrientation(self.body_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(rot)
        # Se altura < 0.05 ou inclinação > 45 graus (0.78 rad)
        if pos[2] < 0.05 or abs(roll) > 0.78 or abs(pitch) > 0.78:
            return True
        return False

    # --- ESTA TAMBÉM FALTAVA ---
    def execute_repair(self):
        # 1. Congela a física (Massa 0) para a gravidade não puxar
        p.changeDynamics(self.body_id, -1, mass=0)
        p.resetBaseVelocity(self.body_id, [0,0,0], [0,0,0])
        
        # 2. Teleporta para a BASE (Respawn Seguro)
        p.resetBasePositionAndOrientation(self.body_id, config.BASE_POS, [0,0,0,1])
        
        # 3. Cor de Alerta (Laranja)
        p.changeVisualShape(self.body_id, -1, rgbaColor=[1, 0.5, 0, 1])
        
        # 4. Zera erros do PID
        self.pid_x.prev_error = 0; self.pid_y.prev_error = 0; self.pid_z.prev_error = 0
        self.pid_x.integral = 0;   self.pid_y.integral = 0;   self.pid_z.integral = 0

    # --- E ESTA ---
    def finish_repair(self):
        # 1. Devolve a Massa 1
        p.changeDynamics(self.body_id, -1, mass=1)
        
        # 2. FORÇA O "ACORDAR" (Wake Up) da física
        p.changeDynamics(self.body_id, -1, activationState=p.ACTIVATION_STATE_WAKE_UP)
        
        # 3. Volta cor Azul
        p.changeVisualShape(self.body_id, -1, rgbaColor=[0, 0, 1, 1])

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
        current_pos, _ = p.getBasePositionAndOrientation(self.body_id)
        # Trava rotação (Giroscópio perfeito)
        p.resetBasePositionAndOrientation(self.body_id, current_pos, [0, 0, 0, 1]) 

        if not self.current_target:
            p.resetBaseVelocity(self.body_id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
            return False 

        target = self.current_target
        dx = target[0] - current_pos[0]; dy = target[1] - current_pos[1]; dz = target[2] - current_pos[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist < config.ARRIVAL_THRESHOLD:
            self.pid_x.prev_error = 0; self.pid_y.prev_error = 0; self.pid_z.prev_error = 0
            return True 

        dt = config.TIME_STEP
        vx = self.pid_x.compute(target[0], current_pos[0], dt)
        vy = self.pid_y.compute(target[1], current_pos[1], dt)
        vz = self.pid_z.compute(target[2], current_pos[2], dt)
        
        limit = config.FLIGHT_SPEED
        vx = max(min(vx, limit), -limit)
        vy = max(min(vy, limit), -limit)
        vz = max(min(vz, limit), -limit)

        p.resetBaseVelocity(self.body_id, linearVelocity=[vx, vy, vz], angularVelocity=[0, 0, 0])
        return False