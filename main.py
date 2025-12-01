import pybullet as p
import time
import config
from modules.environment import Environment
from modules.drone_controller import Drone
from modules.planner import optimize_route
from modules.telemetry import MqttHandler

def main():
    # 1. Inicializa Simulação
    p.connect(p.GUI)
    p.setGravity(0, 0, config.GRAVITY)
    
    # 2. Inicializa Módulos
    env = Environment()
    env.setup_random_targets(count=6, area_size=8) # Cria 6 caixas espalhadas
    
    drone = Drone(config.BASE_POS)
    mqtt = MqttHandler()
    
    # Estado inicial
    mission_complete = False
    
    # Definir um primeiro alvo aleatório para iniciar a patrulha (ex: canto da sala)
    # Ou simplesmente deixar ele parado até detectar algo (se nascer perto de um)
    # Vamos fazê-lo ir para o centro + offset para começar a andar
    drone.current_target = [5, 5, 2] 

    print("--- Simulação Iniciada ---")

    while True:
        p.stepSimulation()
        
        current_pos = drone.get_position()
        
        # --- 1. SENSOR & DETECÇÃO ---
        # Verifica se o sensor do drone "viu" alvos desconhecidos
        # Passamos env.targets para simular a leitura do mundo
        new_targets_objs = drone.scan_for_targets(env.targets)
        
        if new_targets_objs:
            print(f"[EVENTO] Detectados {len(new_targets_objs)} novos alvos!")
            
            # 1.1 Marca no ambiente como 'Visto'
            for t_obj in new_targets_objs:
                env.mark_detected(t_obj['id'])
                # Adiciona à fila de rota do drone se ainda não estiver
                if t_obj['pos'] not in drone.route_queue:
                    drone.route_queue.append(t_obj['pos'])

            # 1.2 REPLANEJAMENTO DINÂMICO (O coração do projeto)
            # Recalcula a ordem de todos os pendentes baseado em onde o drone está AGORA
            drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
            
            # Atualiza o alvo imediato para ser o primeiro da nova lista otimizada
            if drone.route_queue:
                drone.current_target = drone.route_queue[0]

        # --- 2. MOVIMENTAÇÃO & ENTREGA ---
        arrived = drone.move_step()
        
        if arrived:
            if drone.current_target == config.BASE_POS and not drone.route_queue:
                if not mission_complete:
                    print("--- Missão Completa: Retornou à Base ---")
                    mission_complete = True
            
            elif drone.current_target in drone.route_queue:
                print(f"[AÇÃO] Entrega realizada em: {drone.current_target}")
                env.mark_delivered(drone.current_target)
                
                # Remove da fila
                drone.route_queue.remove(drone.current_target)
                
                # Define próximo
                if drone.route_queue:
                    # Re-otimiza novamente? (Opcional, mas bom para garantir)
                    drone.route_queue = optimize_route(current_pos, drone.route_queue, config.BASE_POS)
                    drone.current_target = drone.route_queue[0]
                else:
                    print("[INFO] Todos entregues. Voltando para Base.")
                    drone.current_target = config.BASE_POS

        # --- 3. TELEMETRIA ---
        # Envia dados a cada X frames para não saturar
        # Aqui simplificado enviando sempre (ajuste com contador se ficar lento)
        status_msg = "RETURNING" if (not drone.route_queue and drone.current_target == config.BASE_POS) else "DELIVERING"
        mqtt.send_data(status_msg, current_pos, drone.current_target, len(drone.route_queue))

        time.sleep(config.TIME_STEP)

if __name__ == "__main__":
    main()