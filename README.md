# Rover Digital Twin — ROS 2, Gazebo Harmonic & ESP32

Architettura **Hardware-in-the-Loop (HIL) / Digital Twin** per un rover autonomo a 4 ruote a cinematica differenziale. Il progetto collega una simulazione fisica in **ROS 2 / Gazebo Harmonic** a un microcontrollore **ESP32** programmato in **C puro (ESP-IDF)**, eseguibile sia in emulazione tramite **Wokwi** sia su hardware reale tramite driver motore **L298N**.

---

## Indice
1. [Architettura del Sistema](#architettura-del-sistema)
2. [Struttura del Repository](#struttura-del-repository)
3. [Mappatura Pin Hardware (ESP32 ↔ L298N / Wokwi)](#mappatura-pin-hardware-esp32--l298n--wokwi)
4. [Prerequisiti](#prerequisiti)
5. [Guida all'Avvio (Simulazione Completa)](#guida-allavvio-simulazione-completa)
6. [Passaggio al Rover Reale](#passaggio-al-rover-reale)

---

## Architettura del Sistema

Il flusso dati in tempo reale segue quattro stadi:

1. **Ambiente Fisico (`Gazebo Harmonic`)**: simula il rover in `arena.sdf` con due sensori LiDAR (`/front_sensor/scan` e `/rear_sensor/scan`) e attuazione differenziale (`gz-sim-diff-drive-system`).
2. **Navigazione Autonoma (`rover_controller.cpp`)**: nodo ROS 2 in C++ che elabora le scansioni LiDAR tramite un algoritmo **Braitenberg** longitudinale per l'evitamento ostacoli, pubblicando comandi di velocità su `/cmd_vel` (`geometry_msgs/msg/Twist`).
3. **Bridge Seriale (`wokwi_bridge.py`)**: sottoscrive `/cmd_vel` e i topic LiDAR, inviando via UART (`rfc2217://localhost:4000` oppure `/dev/ttyUSB0`) pacchetti ASCII nel formato:
   ```text
   <linear_x>,<angular_z>,<obstacle_flag>\n
   ```
4. **Firmware Embedded (`main.c` — ESP-IDF)**: riceve i pacchetti via `UART0` (`115200 baud`), calcola la ripartizione differenziale tra ruote sinistre e destre e pilota i canali PWM (`LEDC`) e i pin direzionali del ponte H **L298N**.

---

## Struttura del Repository

```text
digitalTwin_rover/
├── README.md
├── .gitignore
├── ros2_ws/                            # Workspace ROS 2 (gestito tramite Pixi)
│   ├── pixi.toml                       # Dipendenze ROS 2, Gazebo Harmonic e Python
│   ├── pixi.lock
│   └── src/
│       └── my_rover/
│           ├── launch/
│           │   └── sim.launch.py       # Avvio Gazebo, bridge ROS-GZ, controller e Wokwi bridge
│           ├── scripts/
│           │   └── wokwi_bridge.py     # Nodo bridge seriale (/cmd_vel e /scan -> ESP32)
│           ├── src/
│           │   └── rover_controller.cpp # Controller autonomo Braitenberg in C++
│           ├── urdf/
│           │   └── rover.urdf.xacro    # Modello cinematico, inerzie, sensori LiDAR e diff-drive
│           └── worlds/
│               └── arena.sdf           # Ambiente simulato con ostacoli
└── esp32_digital_twin/                 # Progetto firmware ESP32 (PlatformIO + ESP-IDF + Wokwi)
    └── digital_twin/
        ├── platformio.ini              # Configurazione build ESP-IDF per esp32dev
        ├── wokwi.toml                  # Configurazione simulatore e porta seriale TCP (4000)
        ├── diagram.json                # Schema circuitale Wokwi (ESP32 + 5 LED diagnostici)
        └── src/
            └── main.c                  # Firmware C puro: UART parser, cinematica e PWM LEDC
```

---

## Mappatura Pin Hardware (ESP32 ↔ L298N / Wokwi)

| Segnale | Pin ESP32 | Funzione | Feedback Visivo Wokwi (`diagram.json`) |
| :--- | :---: | :--- | :--- |
| `ENA` | `GPIO 21` | Segnale PWM velocità motore sinistro (`LEDC_CH0`) | — |
| `IN1` | `GPIO 19` | Direzione avanti motore sinistro | LED Verde (`SX AVANTI`) |
| `IN2` | `GPIO 18` | Direzione indietro motore sinistro | LED Giallo (`SX INDIETRO`) |
| `IN3` | `GPIO 17` | Direzione avanti motore destro | LED Verde (`DX AVANTI`) |
| `IN4` | `GPIO 16` | Direzione indietro motore destro | LED Giallo (`DX INDIETRO`) |
| `ENB` | `GPIO 4` | Segnale PWM velocità motore destro (`LEDC_CH1`) | — |
| `OBS` | `GPIO 2` | Indicatore ostacolo rilevato sotto soglia (`< 0.8 m`) | LED Rosso (`OSTACOLO`) |

---

## Prerequisiti

* **Sistema Operativo**: Ubuntu Linux (24.04 / 26.04)
* **Gestore Ambienti ROS 2**: [Pixi](https://pixi.sh/) installato nel sistema
* **Editor & Toolchain Embedded**:
  * Visual Studio Code
  * Estensione **PlatformIO IDE**
  * Estensione **Wokwi Simulator** (oppure emulatore locale QEMU per ESP32)

---

## Guida all'Avvio (Simulazione Completa)

Per sincronizzare correttamente la porta seriale virtuale (`localhost:4000`), avvia **prima** il simulatore ESP32 e **poi** il workspace ROS 2.

### Fase 1 — Avvio del Firmware ESP32 su Wokwi

1. Apri la cartella `esp32_digital_twin/digital_twin` in Visual Studio Code.
2. Compila il firmware ESP-IDF cliccando sull'icona **Build** (`✓`) nella barra inferiore di PlatformIO oppure premendo **`Ctrl + Alt + B`**, verificando che venga generato il file `.pio/build/esp32dev/firmware.elf`.
3. Apri il file **`diagram.json`** nell'editor di VS Code.
4. Clicca sul pulsante verde **Play** in alto a sinistra nella finestra del diagramma Wokwi: la simulazione si avvierà esponendo la seriale `UART0` sulla porta `localhost:4000` (come definito in `wokwi.toml`).

### Fase 2 — Compilazione e Avvio di ROS 2 + Gazebo

1. Apri un terminale nella cartella `ros2_ws` (oppure usa la finestra VS Code del workspace ROS 2).
2. Al primo avvio, installa tutte le dipendenze dichiarate in `pixi.toml`:
   ```bash
   pixi install
   ```
3. Compila il pacchetto `my_rover` e avvia la simulazione completa tramite launch file:
   ```bash
   pixi run bash -c "colcon build --packages-select my_rover --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash && ros2 launch my_rover sim.launch.py"
   ```
   *(Se hai configurato il task predefinito in `.vscode/tasks.json`, puoi eseguire questo passaggio direttamente con **`Ctrl + Shift + B`**).*

### Verifica del Funzionamento
* Nel terminale ROS 2, il nodo `wokwi_bridge.py` stamperà:
  ```text
  [INFO] [wokwi_bridge]: Connesso a Wokwi su rfc2217://localhost:4000
  ```
* Nella finestra di **Gazebo**, il rover inizia a muoversi evitando autonomamente gli ostacoli dell'arena.
* Nella finestra di **Wokwi**, i 4 LED direzionali (`SX AVANTI`, `SX INDIETRO`, `DX AVANTI`, `DX INDIETRO`) commutano in sincronia con le sterzate del rover, mentre il LED rosso `OSTACOLO` si accende ogni volta che un oggetto entra sotto `0.8 m` dai sensori LiDAR.

---

## Passaggio al Rover Reale

Lo stesso codice C in `esp32_digital_twin/digital_twin/src/main.c` può essere eseguito senza modifiche sull'hardware fisico (ESP32 + driver L298N + batteria LiPo 2S):

1. Collega la scheda ESP32 al PC tramite cavo USB e carica il firmware con PlatformIO (**Upload**).
2. Avvia il nodo bridge ROS 2 sostituendo l'URL di Wokwi con la porta seriale fisica:
   ```bash
   ros2 run my_rover wokwi_bridge.py --ros-args -p wokwi_url:=/dev/ttyUSB0
   ```