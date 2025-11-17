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
  Serial.println("System Ready. Type 'led:on' or 'motor:200:1'");
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
       
       // Set Direction (Always one way for test)
       digitalWrite(DIR_PIN, HIGH);
       
       // Step the motor
       for(int i=0; i<steps; i++) {
         digitalWrite(STEP_PIN, HIGH);
         delay(10); // Speed of motor
         digitalWrite(STEP_PIN, LOW);
         delay(10);
       }
    }
  }
}