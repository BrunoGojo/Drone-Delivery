import pybullet as p
import time
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
    # Usando 100 alvos conforme seu código anterior
    env.setup_smart_targets(count=100, area_size=15) 
    
    drone = Drone(config.BASE_POS)
    mqtt = MqttHandler()
    
    # 3. Gera Rota de Patrulha
    patrol_waypoints = generate_patrol_grid(area_size=16, height=2, step=4)
    current_patrol_index = 0
    
    print(f"--- Simulação Iniciada ---")
    print(f"Waypoints de patrulha gerados: {len(patrol_waypoints)}")

    while True:
        p.stepSimulation()
        current_pos = drone.get_position()
        
        # --- LÓGICA DE ESTADO ---
        if drone.route_queue:
            mode = "DELIVERY"
            # O alvo é o primeiro da fila.
            # Graças à lógica abaixo, a fila só é reordenada quando completamos uma tarefa.
            drone.current_target = drone.route_queue[0]
            
        elif current_patrol_index < len(patrol_waypoints):
            mode = "PATROL"
            drone.current_target = patrol_waypoints[current_patrol_index]
            
        else:
            mode = "RETURNING"
            drone.current_target = config.BASE_POS

        # --- 1. SENSOR & DETECÇÃO ---
        visible_targets = drone.scan_for_targets(env.targets)
        
        if visible_targets:
            new_batch = visible_targets[:2] 
            
            detected_new = False
            for t_obj in new_batch:
                env.mark_detected(t_obj['id']) 
                
                if t_obj['pos'] not in drone.route_queue:
                    # REGRA 3: Apenas armazena!
                    drone.route_queue.append(t_obj['pos'])
                    detected_new = True
            
            if detected_new:
                # REGRA 3: O drone NÃO interrompe a missão atual.
                # Removemos o optimize_route daqui.
                print(f"[MEMÓRIA] Novos pontos salvos na fila. Mantendo rota atual...")

        # --- 2. MOVIMENTAÇÃO ---
        arrived = drone.move_step()
        
        if arrived:
            if mode == "DELIVERY":
                print(f"[AÇÃO] Entrega realizada em: {drone.current_target}")
                env.mark_delivered(drone.current_target)
                
                # Remove o ponto atual da fila
                drone.route_queue.pop(0) 
                
                # REGRA 4: Replanejamento parcial
                # Agora que terminou a tarefa atual, ele reavalia o futuro.
                if drone.route_queue:
                    print("[CÉREBRO] Entrega concluída. Recalculando melhor rota para os restantes...")
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)

            elif mode == "PATROL":
                current_patrol_index += 1
                
                # Se durante essa perna da patrulha ele coletou pontos na memória,
                # agora é a hora de otimizar antes de começar o modo DELIVERY no próximo loop.
                if drone.route_queue:
                    print("[CÉREBRO] Fim do trecho de patrulha. Iniciando entregas otimizadas.")
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
            
            elif mode == "RETURNING":
                print("--- Missão Completa: Drone pousado ---")
                time.sleep(2)
                break

        # --- 3. TELEMETRIA ---
        mqtt.send_data(mode, current_pos, drone.current_target, len(drone.route_queue))

        time.sleep(config.TIME_STEP)

if __name__ == "__main__":
    main()