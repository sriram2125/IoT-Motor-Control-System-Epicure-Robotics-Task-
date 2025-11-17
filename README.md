# IoT Motor Control System (Epicure Robotics Task)

This repository contains the complete software solution for the **Robotic Software Engineer Task** at Epicure Robotics. 

The system implements a full communication pipeline where a **Python** script on a computer controls a **Stepper Motor** and **LED** connected to an **STM32** microcontroller, bridged wirelessly via an **ESP32** over **MQTT**.

## 📌 Project Overview
[cite_start]**Objective:** Develop a Python-based communication system interfacing between a PC, an ESP microcontroller (via MQTT), and an STM microcontroller (via UART) to control hardware[cite: 4, 5].

### System Architecture
[cite_start]The data flow follows the architecture defined in the task requirements [cite: 8-12]:
1.  **PC (Python):** Captures user commands and publishes to MQTT topic `epicure/commands`.
2.  **ESP32 (Bridge):** Subscribes to MQTT, receives the message, and forwards it via UART (Serial2).
3.  **STM32 (Controller):** Listens on UART, parses the command string, and executes motor/LED control logic.

```mermaid
graph LR
    A[User/Python] -- MQTT --> B((Cloud Broker))
    B -- MQTT --> C[ESP32 Bridge]
    C -- UART --> D[STM32 Controller]
    D --> E[Nema17 Stepper & LED]

-----

##🛠️ Implementation & Simulation Strategy

Due to hardware constraints, this project utilizes a **Hybrid Simulation Strategy** using [Wokwi](https://wokwi.com). The system was validated in two integrated stages to ensure full functionality without physical wiring.

### 1\. Communication Stack (Python ↔ ESP32)

  * **Tools:** macOS Terminal (Python) + Wokwi ESP32 Simulator.
  * **Validation:** Confirmed that strings sent from the Python script via `paho-mqtt` are instantaneously received by the ESP32 and printed to the Serial Monitor/UART buffer.

### 2\. Control Logic Stack (STM32 ↔ Hardware)

  * **Tools:** Wokwi STM32 Nucleo Simulator + A4988 Driver + Nema 17 Stepper.
  * **Validation:** Validated UART parsing logic by injecting command strings directly into the serial stream. Confirmed precise motor stepping and LED toggling.

-----

## 📸 Testing & Results

### 1\. Full System Communication Test

*Below: Python running on macOS Terminal (Left) sending `motor:200:1` to the Virtual ESP32 (Right) over the public MQTT broker. The message is successfully received and forwarded.*

### 2\. STM32 Motor Control Verification

*Below: The STM32 Nucleo receiving the parsed UART command and driving the Nema 17 Stepper Motor via the A4988 Driver.*

-----

## 📂 Repository Structure

```text
Epicure_Robotics_Task/
├── 1_Python_Control/
│   ├── main.py              # The control station script
│   └── requirements.txt     # Dependencies (paho-mqtt)
├── 2_ESP32_Firmware/
│   └── esp32_firmware.ino   # MQTT Subscriber & UART Forwarder
├── 3_STM32_Firmware/
│   └── stm32_firmware.ino   # UART Listener & Motor Controller
├── assets/                  # Screenshots and diagrams
└── README.md                # Documentation
```

-----

## 🚀 Getting Started

### Prerequisites

  * **Python 3.x**
  * **Arduino IDE** (for viewing firmware)
  * **Wokwi Simulator** (Web-based)

### 1\. Python Setup (The Controller)

Navigate to the `1_Python_Control` directory and install dependencies:

```bash
cd 1_Python_Control
pip install -r requirements.txt
```

Run the controller:

```bash
python3 main.py
```

### 2\. ESP32 Firmware (The Bridge)

  * **Source:** `2_ESP32_Firmware/esp32_firmware.ino`
  * **Configuration:** \* Set `ssid` to `"Wokwi-GUEST"` (for simulation) or your credentials.
      * Set `mqtt_server` to `"broker.hivemq.com"` or `"test.mosquitto.org"`.
  * [cite\_start]**Logic:** Listens on `epicure/commands` and `Serial2.print()` to UART [cite: 25-28].

### 3\. STM32 Firmware (The Driver)

  * **Source:** `3_STM32_Firmware/stm32_firmware.ino`
  * **Hardware Target:** STM32 Nucleo C031C6 (or F407VET6 as per docs).
  * **Pinout (Nucleo Simulation):**
      * **UART:** Default Serial (USB/Virtual).
      * **Stepper Step:** D3
      * **Stepper Dir:** D2
      * **LED:** D13
  * [cite\_start]**Logic:** Reads serial buffer until newline `\n`, then executes [cite: 29-30].

-----

## 📡 Communication Protocol

The system uses a strict string-based protocol for commands.

| Command Type | Format | Example | Description |
| :--- | :--- | :--- | :--- |
| **Motor Control** | `motor:<steps>:<dir>` | `motor:200:1` | Moves stepper 200 steps in direction 1. |
| **LED Control** | `led:<state>` | `led:on` | Turns the LED High/Low. |

-----

