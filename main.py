import pybullet as p
import time
import config
from modules.environment import Environment
from modules.drone_controller import Drone
from modules.planner import optimize_route, generate_patrol_grid, calculate_distance
from modules.telemetry import MqttHandler

def main():
    # 1. Inicializa Simulação
    p.connect(p.GUI)
    p.setGravity(0, 0, config.GRAVITY)
    
    # 2. Configura Ambiente
    env = Environment()
    # Aumentei para 15 alvos espalhados em uma área de 10x10
    env.setup_smart_targets(count=100, area_size=20) 
    
    drone = Drone(config.BASE_POS)
    mqtt = MqttHandler()
    
    # 3. Gera Rota de Patrulha (Grid de busca)
    # O drone vai voar nesses pontos para tentar achar as caixas
    patrol_waypoints = generate_patrol_grid(area_size=21, height=2, step=4)
    current_patrol_index = 0
    
    print(f"--- Simulação Iniciada ---")
    print(f"Waypoints de patrulha gerados: {len(patrol_waypoints)}")

    while True:
        p.stepSimulation()
        current_pos = drone.get_position()
        
        # --- LÓGICA DE ESTADO ---
        # Prioridade 1: Se tem entregas na fila, FAÇA AS ENTREGAS (Modo Delivery)
        if drone.route_queue:
            mode = "DELIVERY"
            drone.current_target = drone.route_queue[0]
            
        # Prioridade 2: Se não tem entregas, CONTINUE A PATRULHA (Modo Patrol)
        elif current_patrol_index < len(patrol_waypoints):
            mode = "PATROL"
            drone.current_target = patrol_waypoints[current_patrol_index]
            
        # Prioridade 3: Acabou a patrulha e as entregas, VOLTE PRA BASE
        else:
            mode = "RETURNING"
            drone.current_target = config.BASE_POS

        # --- 1. SENSOR & DETECÇÃO ---
        # Escaneia o ambiente
        visible_targets = drone.scan_for_targets(env.targets)
        
        if visible_targets:
            # REGRA DO USUÁRIO: Detectar no máximo 2 pontos novos por vez
            # Se achou 5, pega só os 2 primeiros para simular "capacidade do sensor"
            new_batch = visible_targets[:2] 
            
            print(f"[EVENTO] Sensor ativo! Detectados {len(new_batch)} alvos neste ponto.")
            
            detected_any = False
            for t_obj in new_batch:
                env.mark_detected(t_obj['id']) # Muda cor no PyBullet
                
                # Só adiciona se ainda não estava na fila
                if t_obj['pos'] not in drone.route_queue:
                    drone.route_queue.append(t_obj['pos'])
                    detected_any = True

            # Se detectou algo novo, OTIMIZA A ROTA IMEDIATAMENTE
            if detected_any:
                drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)

        # --- 2. MOVIMENTAÇÃO ---
        arrived = drone.move_step()
        
        if arrived:
            # Caso A: Chegou num ponto de ENTREGA
            if mode == "DELIVERY":
                print(f"[AÇÃO] Entrega realizada em: {drone.current_target}")
                env.mark_delivered(drone.current_target)
                drone.route_queue.pop(0) # Remove da fila
                
                # Se ainda houver entregas, re-otimiza baseado na nova posição
                if drone.route_queue:
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)

            # Caso B: Chegou num ponto de PATRULHA
            elif mode == "PATROL":
                # Apenas avança para o próximo waypoint de patrulha
                current_patrol_index += 1
            
            # Caso C: Voltou pra Base
            elif mode == "RETURNING":
                print("--- Missão Completa: Drone pousado ---")
                # Reinicia ou encerra (aqui só avisa e dorme)
                time.sleep(2)
                break

        # --- 3. TELEMETRIA ---
        # Envia dados para o Node-RED
        mqtt.send_data(mode, current_pos, drone.current_target, len(drone.route_queue))

        time.sleep(config.TIME_STEP)

if __name__ == "__main__":
    main()