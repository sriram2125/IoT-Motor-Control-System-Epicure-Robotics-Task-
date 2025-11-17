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
