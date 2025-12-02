# Configurações de Simulação
TIME_STEP = 1./240.
GRAVITY = -9.8

# Configurações do Drone
DETECTION_RADIUS = 3.0  # Raio do sensor (metros)
FLIGHT_SPEED = 6.0      # Velocidade de deslocamento
ARRIVAL_THRESHOLD = 0.4 # Distância para considerar "chegou"

# Posições (X, Y, Z)
BASE_POS = [0, 0, 1]    # Onde o drone nasce e para onde volta

# Configurações MQTT (Node-RED)
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_TELEMETRY = "drone/telemetry"

# ... (mantenha o código anterior)

# ... (Mantenha o resto igual)

# Configurações do Controlador PID (Estabilidade)
# Ajustado para ser mais suave e não causar oscilação
PID_GAINS = {
    'kp': 3.8,  # Era 4.0 (Muito forte). 1.5 é mais suave.
    'ki': 0.0,  # Era 0.02. Zeramos para evitar efeito elástico.
    'kd': 0.1   # Amortecimento para frear ao chegar.
}