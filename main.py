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
    
    # --- VARIÁVEIS DE ESTATÍSTICA ---
    start_time = time.time()
    total_distance = 0.0
    ideal_distance_accumulated = 0.0
    last_stop_pos = config.BASE_POS 
    
    energy_consumed = 0.0
    delivered_count = 0
    replan_count = 0
    
    # Históricos para Estabilidade
    last_frame_pos = config.BASE_POS
    z_history = []   
    vel_history = [] 
    
    # Rota Inicial
    patrol_waypoints = generate_patrol_grid(area_size=13, height=2, step=4)
    current_patrol_index = 0
    
    # CONTADOR PARA REDUZIR LAG (Anti-Flood)
    frame_counter = 0
    
    print("--- Supervisório Iniciado: Modo Otimizado ---")

    while True:
        p.stepSimulation()
        current_pos = drone.get_position()
        frame_counter += 1
        
        # --- CÁLCULOS FÍSICOS (A cada frame) ---
        step_dist = math.sqrt((current_pos[0] - last_frame_pos[0])**2 + (current_pos[1] - last_frame_pos[1])**2)
        total_distance += step_dist
        energy_consumed += (step_dist * 5.0) + 0.05
        
        # Estabilidade (Z e Velocidade)
        z_history.append(current_pos[2])
        if len(z_history) > 50: z_history.pop(0)
        
        current_vel = step_dist / config.TIME_STEP
        vel_history.append(current_vel)
        if len(vel_history) > 50: vel_history.pop(0)
        
        last_frame_pos = current_pos

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

        # --- SENSOR ---
        visible_targets = drone.scan_for_targets(env.targets)
        if visible_targets:
            new_batch = visible_targets[:2] 
            detected_new = False
            for t_obj in new_batch:
                env.mark_detected(t_obj['id']) 
                if t_obj['pos'] not in drone.route_queue:
                    drone.route_queue.append(t_obj['pos'])
                    detected_new = True
            if detected_new:
                # print removido para economizar terminal
                pass

        # --- MOVIMENTAÇÃO ---
        arrived = drone.move_step()
        
        if arrived:
            if mode == "DELIVERY":
                print(f"[AÇÃO] Entrega #{delivered_count+1} feita.")
                env.mark_delivered(drone.current_target)
                
                dist_ideal_step = math.sqrt((current_pos[0] - last_stop_pos[0])**2 + 
                                            (current_pos[1] - last_stop_pos[1])**2)
                                            
                ideal_distance_accumulated += dist_ideal_step
                last_stop_pos = current_pos # Atualiza para a posição real atual
                
                delivered_count += 1
                drone.route_queue.pop(0) 
                
                if drone.route_queue:
                    print("[CÉREBRO] Otimizando rota...")
                    replan_count += 1
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)

            elif mode == "PATROL":
                current_patrol_index += 1
                if drone.route_queue:
                    replan_count += 1
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
            
            elif mode == "RETURNING":
                print(f"--- Fim. Pouso Confirmado ---")
                break

        # --- TELEMETRIA OTIMIZADA (A cada 10 frames = 0.1s) ---
        if frame_counter % 10 == 0:
            elapsed_time = time.time() - start_time
            avg_time_per_point = (elapsed_time / delivered_count) if delivered_count > 0 else 0.0
            efficiency = (ideal_distance_accumulated / total_distance * 100) if total_distance > 0.1 else 100.0
            
            # Converte numpy floats para python floats para evitar erro JSON
            stability_z = float(np.std(z_history)) if len(z_history) > 1 else 0.0
            stability_v = float(np.std(vel_history)) if len(vel_history) > 1 else 0.0

            payload = {
                "state": mode,
                "position": {"x": round(current_pos[0], 2), "y": round(current_pos[1], 2)},
                "stats": {
                    "dist": round(total_distance, 2),
                    "energy": round(energy_consumed, 1),
                    "replan": replan_count,
                    "pending": len(drone.route_queue),
                    "efficiency": round(efficiency, 1),
                    "avg_time": round(avg_time_per_point, 1),
                    "stability_v": round(stability_v, 2)
                }
            }
            mqtt.client.publish(config.MQTT_TOPIC_TELEMETRY, str(payload).replace("'", '"'))

        # Mantém a simulação fluida
        time.sleep(config.TIME_STEP)

if __name__ == "__main__":
    main()