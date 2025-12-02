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
    p.connect(p.GUI)
    p.setGravity(0, 0, config.GRAVITY)
    
    env = Environment()
    env.setup_smart_targets(count=100, area_size=16) 
    
    drone = Drone(config.BASE_POS)
    mqtt = MqttHandler()
    
    # Métricas
    start_time = time.time()
    total_distance = 0.0
    ideal_distance_accumulated = 0.0
    last_stop_pos = config.BASE_POS 
    energy_consumed = 0.0
    delivered_count = 0
    replan_count = 0
    
    # --- NOVO: Histórico de Pontos Visitados ---
    visited_log = [] # Lista de coordenadas [x, y] das entregas feitas
    
    # Históricos de Física
    last_frame_pos = config.BASE_POS
    z_history = [] 
    vel_history = [] 
    
    # Rota
    patrol_waypoints = generate_patrol_grid(area_size=17, height=2, step=4)
    current_patrol_index = 0
    frame_counter = 0

    # Variáveis de Reparo
    repair_mode = False
    repair_start_time = 0
    
    print("--- Supervisório: Mapa com Histórico Ativo ---")

    while True:
        p.stepSimulation()
        current_pos = drone.get_position()
        frame_counter += 1
        
        # --- 1. DETECÇÃO DE QUEDA ---
        if not repair_mode:
            if drone.check_crash():
                print(f"[ALERTA] CRASH! Drone caiu. Respawnando na base em 3s...")
                repair_mode = True
                repair_start_time = time.time()
                drone.execute_repair() 
                energy_consumed += 100.0

        # --- LÓGICA DE ESTADO ---
        if repair_mode:
            mode = "REPAIR"
            tempo_passado = time.time() - repair_start_time
            if frame_counter % 60 == 0: 
                print(f"Reparando... {3.0 - tempo_passado:.1f}s")
                
            if tempo_passado > 3.0:
                print("[SISTEMA] Sistema Reiniciado. Decolando!")
                drone.finish_repair()
                repair_mode = False
                drone.current_target = config.BASE_POS
        else:
            # Lógica Normal
            if drone.route_queue:
                mode = "DELIVERY"
                drone.current_target = drone.route_queue[0]
            elif current_patrol_index < len(patrol_waypoints):
                mode = "PATROL"
                drone.current_target = patrol_waypoints[current_patrol_index]
            else:
                mode = "RETURNING"
                drone.current_target = config.BASE_POS

            # Sensor
            visible_targets = drone.scan_for_targets(env.targets)
            if visible_targets:
                new_batch = visible_targets[:2] 
                for t_obj in new_batch:
                    env.mark_detected(t_obj['id']) 
                    if t_obj['pos'] not in drone.route_queue:
                        drone.route_queue.append(t_obj['pos'])

            # Movimento
            arrived = drone.move_step()
            
            # Métricas Físicas
            step_dist = math.sqrt((current_pos[0] - last_frame_pos[0])**2 + (current_pos[1] - last_frame_pos[1])**2)
            total_distance += step_dist
            energy_consumed += (step_dist * 5.0) + 0.05
            last_frame_pos = current_pos
            
            if arrived:
                if mode == "DELIVERY":
                    print(f"[AÇÃO] Entrega #{delivered_count+1} feita.")
                    env.mark_delivered(drone.current_target)
                    
                    # --- NOVO: Adiciona ao log de visitados ---
                    # Salvamos apenas X e Y para economizar dados
                    visited_log.append([round(drone.current_target[0], 2), round(drone.current_target[1], 2)])
                    
                    dist_ideal_step = math.sqrt((current_pos[0] - last_stop_pos[0])**2 + (current_pos[1] - last_stop_pos[1])**2)
                    ideal_distance_accumulated += dist_ideal_step
                    last_stop_pos = current_pos 
                    delivered_count += 1
                    drone.route_queue.pop(0) 
                    if drone.route_queue:
                        replan_count += 1
                        drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
                elif mode == "PATROL":
                    current_patrol_index += 1
                    if drone.route_queue:
                        replan_count += 1
                        drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
                elif mode == "RETURNING":
                    print(f"--- Fim da Missão ---")
                    break

        # --- TELEMETRIA ---
        z_history.append(current_pos[2])
        if len(z_history) > 50: z_history.pop(0)
        
        current_vel = 0.0 if repair_mode else (math.sqrt((current_pos[0]-last_frame_pos[0])**2)/config.TIME_STEP)
        vel_history.append(current_vel)
        if len(vel_history) > 50: vel_history.pop(0)

        if frame_counter % 10 == 0:
            elapsed_time = time.time() - start_time
            avg_time = (elapsed_time / delivered_count) if delivered_count > 0 else 0.0
            efficiency = (ideal_distance_accumulated / total_distance * 100) if total_distance > 0.1 else 100.0
            
            stability_z = float(np.std(z_history)) if len(z_history) > 1 else 0.0
            stability_v = float(np.std(vel_history)) if len(vel_history) > 1 else 0.0

            payload = {
                "state": mode, 
                "position": {"x": round(current_pos[0], 2), "y": round(current_pos[1], 2)},
                # --- NOVO: Enviamos a lista de visitados dentro do objeto MAP ---
                "map": {
                    "visited": visited_log
                },
                "stats": {
                    "dist": round(total_distance, 2),
                    "energy": round(energy_consumed, 1),
                    "replan": replan_count,
                    "pending": len(drone.route_queue),
                    "efficiency": round(efficiency, 1),
                    "avg_time": round(avg_time, 1),
                    "stability_v": round(stability_v, 2)
                }
            }
            mqtt.client.publish(config.MQTT_TOPIC_TELEMETRY, str(payload).replace("'", '"'))

        time.sleep(config.TIME_STEP)

if __name__ == "__main__":
    main()