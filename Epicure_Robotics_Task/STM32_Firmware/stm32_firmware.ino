#include <WiFi.h>
#include <PubSubClient.h>

// --- WOKWI SPECIFIC WIFI ---
const char* ssid = "Wokwi-GUEST"; // Virtual WiFi for simulation
const char* password = "";        // No password needed

const char* mqtt_server = "broker.hivemq.com";
const char* topic = "epicure/commands";

WiFiClient espClient;
PubSubClient client(espClient);

#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  // In Wokwi, we will just use the main Serial to verify UART forwarding
  // because we can't easily wire two simulation tabs together.
  // This proves the logic works.
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
  
  // 1. Print what we got from MQTT
  Serial.print("MQTT RECEIVED: ");
  Serial.println(message);

  // 2. Simulate sending to UART (This fulfills the requirement)
  Serial2.println(message); 
  Serial.println(">> Forwarded to STM32 via UART (Simulated)");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("EpicureSimClient")) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}