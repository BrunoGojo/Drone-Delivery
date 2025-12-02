import pybullet as p
import time
import math
import numpy as np
import config
from modules.environment import Environment
from modules.drone_controller import Drone
from modules.planner import optimize_route, generate_patrol_grid
from modules.telemetry import MqttHandler

def main():
    # 1. Inicializa Simulação
    p.connect(p.GUI)
    p.setGravity(0, 0, config.GRAVITY)
    
    # 2. Configura Ambiente
    env = Environment()
    env.setup_smart_targets(count=100, area_size=12) 
    
    drone = Drone(config.BASE_POS)
    mqtt = MqttHandler()
    
    # 3. Variáveis de Estatística (MÉTRICAS)
    start_time = time.time()
    total_distance = 0.0
    energy_consumed = 0.0
    replan_count = 0
    last_pos = config.BASE_POS
    z_history = [] 
    
    # Rota Inicial
    patrol_waypoints = generate_patrol_grid(area_size=13, height=2, step=4)
    current_patrol_index = 0
    
    print("--- Supervisório Iniciado: Logs Ativados ---")

    while True:
        p.stepSimulation()
        current_pos = drone.get_position()
        
        # --- CÁLCULO DE MÉTRICAS EM TEMPO REAL ---
        step_dist = math.sqrt((current_pos[0] - last_pos[0])**2 + (current_pos[1] - last_pos[1])**2)
        total_distance += step_dist
        last_pos = current_pos
        
        # Energia: 5J por metro + 0.01J basal
        energy_consumed += (step_dist * 5.0) + 0.01 
        
        # Estabilidade (Variação de Z)
        z_history.append(current_pos[2])
        if len(z_history) > 50: z_history.pop(0)
        # Convertendo para float nativo do Python para não quebrar o JSON
        stability_score = float(np.std(z_history)) if len(z_history) > 1 else 0.0

        # --- LÓGICA DE ESTADO ---
        if drone.route_queue:
            mode = "DELIVERY"
            drone.current_target = drone.route_queue[0]
            
        elif current_patrol_index < len(patrol_waypoints):
            mode = "PATROL"
            drone.current_target = patrol_waypoints[current_patrol_index]
            
        else:
            mode = "RETURNING"
            drone.current_target = config.BASE_POS

        # --- SENSOR & DETECÇÃO ---
        visible_targets = drone.scan_for_targets(env.targets)
        if visible_targets:
            new_batch = visible_targets[:2] 
            detected_new = False
            for t_obj in new_batch:
                env.mark_detected(t_obj['id']) 
                if t_obj['pos'] not in drone.route_queue:
                    drone.route_queue.append(t_obj['pos'])
                    detected_new = True
            
            # [LOG] MEMÓRIA
            if detected_new:
                print(f"[MEMÓRIA] Novos pontos salvos na fila. Mantendo rota atual...")

        # --- MOVIMENTAÇÃO ---
        arrived = drone.move_step()
        
        if arrived:
            if mode == "DELIVERY":
                # [LOG] AÇÃO
                print(f"[AÇÃO] Entrega realizada em: {drone.current_target}")
                
                env.mark_delivered(drone.current_target)
                drone.route_queue.pop(0) 
                
                # REPLANEJAMENTO
                if drone.route_queue:
                    # [LOG] CÉREBRO
                    print("[CÉREBRO] Entrega concluída. Recalculando melhor rota para os restantes...")
                    
                    replan_count += 1
                    # O print [PLANNER] virá de dentro desta função optimize_route automaticamente
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)

            elif mode == "PATROL":
                current_patrol_index += 1
                if drone.route_queue:
                    print("[CÉREBRO] Patrulha interrompida. Iniciando entregas...")
                    replan_count += 1
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
            
            elif mode == "RETURNING":
                print("--- Missão Completa: Pouso Confirmado ---")
                break

        # --- TELEMETRIA COMPLETA ---
        elapsed_time = time.time() - start_time
        
        payload = {
            "state": mode,
            "position": {"x": current_pos[0], "y": current_pos[1], "z": current_pos[2]},
            "stats": {
                "dist": round(total_distance, 2),
                "energy": round(energy_consumed, 1),
                "replan": replan_count,
                "time": round(elapsed_time, 0),
                "stability": round(stability_score, 4),
                "pending": len(drone.route_queue)
            }
        }
        
        mqtt.client.publish(config.MQTT_TOPIC_TELEMETRY, str(payload).replace("'", '"'))

        time.sleep(0.01) # Pequena pausa para estabilidade do MQTT

if __name__ == "__main__":
    main()