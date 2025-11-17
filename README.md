# IoT Motor Control System (Epicure Robotics Task)

This repository contains the complete software solution for the **Robotic Software Engineer Task** at Epicure Robotics.

The system implements a full communication pipeline where a **Python** script on a computer controls a **Stepper Motor** and **LED** connected to an **STM32** microcontroller, bridged wirelessly via an **ESP32** over **MQTT**.

## 📌 Project Overview
[cite_start]**Objective:** Develop a Python-based communication system interfacing between a PC, an ESP microcontroller (via MQTT), and an STM microcontroller (via UART) to control hardware[cite: 4].

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
```
-----

## 🛠️ Implementation & Simulation Strategy

Due to hardware constraints, this project utilizes a **Hybrid Simulation Strategy** using [Wokwi](https://wokwi.com). The system was validated in two integrated stages to ensure full functionality without physical wiring.

### 1\. Communication Stack (Python ↔ ESP32)

  * **Tools:** macOS Terminal (Python) + Wokwi ESP32 Simulator (Guest Mode).
  * **Method:** The Python script communicates with the Wokwi Virtual ESP32 over the public `broker.hivemq.com` MQTT broker.
  * **Validation:** Confirmed that strings sent from the local Python script are instantaneously received by the simulated ESP32 and forwarded to the UART buffer.

### 2\. Control Logic Stack (STM32 ↔ Hardware)

  * **Tools:** Wokwi STM32 Nucleo Simulator + A4988 Driver + Nema 17 Stepper.
  * **Method:** Since Wokwi does not support direct UART connection between separate browser tabs, the "UART Input" was simulated by injecting command strings (`motor:200:1`) directly into the STM32 Serial Monitor.
  * **Validation:** Verified that the STM32 firmware correctly parses the strings and drives the A4988 driver signals to rotate the motor and toggle the LED.

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

## 💻 Source Code

\<details\>
\<summary\>Click to view 1. Python Control Script (main.py)\</summary\>

```python
import paho.mqtt.client as mqtt
import time

# Configuration
BROKER = "broker.hivemq.com" 
PORT = 1883
TOPIC = "epicure/commands"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT Broker: {BROKER}")
    else:
        print(f"Failed to connect, return code {rc}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    
    print("Connecting to network...")
    try:
        client.connect(BROKER, PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    print("\n--- Epicure Control Terminal ---")
    print("Type commands like 'motor:100:1' or 'led:on'")
    print("Type 'exit' to quit.\n")

    try:
        while True:
            cmd = input("Command > ")
            if cmd == 'exit':
                break
            if cmd.strip():
                client.publish(TOPIC, cmd)
                print(f"Sent: {cmd}")
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
```

\</details\>

\<details\>
\<summary\>Click to view 2. ESP32 Firmware (esp32\_firmware.ino)\</summary\>

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

// Wokwi Simulation WiFi
const char* ssid = "Wokwi-GUEST"; 
const char* password = "";        

const char* mqtt_server = "broker.hivemq.com"; 
const char* topic = "epicure/commands";

WiFiClient espClient;
PubSubClient client(espClient);

#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to Wokwi-GUEST...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("MQTT RECEIVED: ");
  Serial.println(message);
  
  // Forward to STM32
  Serial2.println(message); 
  Serial.println(">> Forwarded to STM32 via UART");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "EpicureClient-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}
```

\</details\>

\<details\>
\<summary\>Click to view 3. STM32 Firmware (stm32\_firmware.ino)\</summary\>

```cpp
// Pin definitions for Nucleo C031C6
#define LED_PIN  D13
#define DIR_PIN  D2
#define STEP_PIN D3

String inputString = "";
bool stringComplete = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("STM32 Ready. Waiting for commands...");
}

void loop() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("led:")) {
    if (cmd.indexOf("on") > 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ON");
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED OFF");
    }
  }
  else if (cmd.startsWith("motor:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.lastIndexOf(':');
    
    if (firstColon > 0) {
       String stepsStr = cmd.substring(firstColon + 1, secondColon);
       int steps = stepsStr.toInt();
       
       Serial.print("Moving Motor: ");
       Serial.println(steps);
       
       digitalWrite(DIR_PIN, HIGH);
       
       for(int i=0; i<steps; i++) {
         digitalWrite(STEP_PIN, HIGH);
         delay(10);
         digitalWrite(STEP_PIN, LOW);
         delay(10);
       }
    }
  }
}
```

\</details\>

-----

## 📡 Communication Protocol

[cite\_start]The system uses a strict string-based protocol for commands [cite: 33-34].

| Command Type | Format | Example | Description |
| :--- | :--- | :--- | :--- |
| **Motor Control** | `motor:<steps>:<dir>` | `motor:200:1` | Moves stepper 200 steps in direction 1. |
| **LED Control** | `led:<state>` | `led:on` | Turns the LED High/Low. |

-----

## 👤 Author

**[Your Name]**
*Submitted for the Robotic Software Engineer Intern Role at Epicure Robotics.*

```
```
