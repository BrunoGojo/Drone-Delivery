# 🚁 Drone Delivery System -- Autonomous Logistics Simulation

Simulação avançada de **robótica móvel autônoma**, combinando
planejamento dinâmico de rotas (**TSP**), controle PID multieixo, física
3D realista e supervisão IoT em tempo real via **Node-RED + MQTT**.

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![Node-RED](https://img.shields.io/badge/IoT-Node--RED-8F0000?logo=node-red&logoColor=white)
![PyBullet](https://img.shields.io/badge/Physics-PyBullet-orange)
![MQTT](https://img.shields.io/badge/Protocol-MQTT-660066)
![Status](https://img.shields.io/badge/Status-Completed-success)

------------------------------------------------------------------------

## 📖 Visão Geral

Este projeto implementa um sistema completo de **Logística Autônoma com
Drones**, capaz de:

-   Identificar pacotes no ambiente.
-   Calcular rotas ótimas dinamicamente.
-   Controlar o voo com estabilidade realista.
-   Publicar **telemetria em tempo real** para um dashboard IoT no
    **Node-RED**.

Diferente de simulações estáticas, aqui a física (PyBullet) e o controle
PID trabalham em tempo real enquanto o MQTT transmite constantemente
dados para o supervisório.

------------------------------------------------------------------------

## 🎯 Funcionalidades Principais

### 🧠 Navegação Inteligente

-   Algoritmo **TSP Dinâmico**: Recalcula a melhor rota sempre que novos
    pacotes surgem ou uma entrega termina.

### 🎛 Controle PID 3-Eixos

-   PIDs independentes para X, Y e Z.

### 🌀 Física e Realismo

-   Vibração harmônica simulada.
-   Turbulência e ruído de sensores.
-   Detecção de crashes.

### 🔧 Sistema de Auto-Reparo

-   Reposição automática após quedas.
-   Penalidade de energia.

### 📡 Dashboard IoT em Tempo Real

-   Mapa completo do voo.
-   Rastro azul.
-   Pontos entregues.
-   Gráficos de vibração.

------------------------------------------------------------------------

## 📂 Estrutura do Projeto

    drone_new/
    │
    ├── main.py
    ├── config.py
    ├── requirements.txt
    ├── flows.json
    │
    ├── assets/
    │   └── drone.urdf
    │
    └── modules/
        ├── drone_controller.py
        ├── environment.py
        ├── planner.py
        └── telemetry.py

------------------------------------------------------------------------

## 🛠️ Pré-Requisitos

-   **Python 3.10+**
-   **Mosquitto MQTT Broker**
-   **Node-RED**
-   **PyBullet**

------------------------------------------------------------------------

## 🚀 Instalação

### 1️⃣ MQTT Broker
Verifique se o Mosquitto está rodando:
Windows: Serviços → Mosquitto Broker → Iniciado
Docker: use esta configuração mínima no mosquitto.conf

    listener 1883
    allow_anonymous true

### 2️⃣ Ambiente Python
Abra o terminal na pasta do projeto:

    python -m venv venv
    .env\Scriptsctivate
    pip install -r requirements.txt

### 3️⃣ Node-RED

Instalar `node-red-dashboard`\
Importar `flows.json`\
Se usar Docker → MQTT host: `host.docker.internal`

### 4️⃣ Executar Simulação

    python main.py

------------------------------------------------------------------------

## 📊 O que aparece?

-   Janela PyBullet com drone.
-   Logs no terminal.
-   Dashboard com rota e gráficos.

------------------------------------------------------------------------

## 🐛 Troubleshooting

  Problema                Solução
  ----------------------- ------------------------------------
  `PSSecurityException`   `Set-ExecutionPolicy RemoteSigned`
  Node-RED connecting     Use `host.docker.internal`
  Gráficos zerados        Ativar vibração no controller
  Drone parado            Verificar PyBullet
  Sem auto-reparo         Ativar wake-up na física

------------------------------------------------------------------------

## 📜 Licença

Uso livre educacional.

------------------------------------------------------------------------
