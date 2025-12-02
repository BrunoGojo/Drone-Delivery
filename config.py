# Configurações de Simulação
TIME_STEP = 1./240.
GRAVITY = -9.8

# Configurações do Drone
DETECTION_RADIUS = 3.0  # Raio do sensor (metros)
FLIGHT_SPEED = 8.0      # Velocidade de deslocamento
ARRIVAL_THRESHOLD = 0.5 # Distância para considerar "chegou"

# Posições (X, Y, Z)
BASE_POS = [0, 0, 1]    # Onde o drone nasce e para onde volta

# Configurações MQTT (Node-RED)
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_TELEMETRY = "drone/telemetry"