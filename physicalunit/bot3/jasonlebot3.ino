#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi gegevens
const char* wifiSSID = "PLACEHOLDERwifinaam";        // Vul hier je WiFi-netwerknaam in
const char* wifiPassword = "PLACEHOLDERwachtwoord";  // Vul hier je WiFi-wachtwoord in

// MQTT configuratie
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* subscribeTopic = "robot/status";
const char* commandTopic = "robot/command";

// LED pins
#define LED_NORTH 14
#define LED_EAST 25
#define LED_SOUTH 26
#define LED_WEST 27

// Emergency Stop Button
#define BUTTON_STOP 33

WiFiClient espClient;
PubSubClient client(espClient);

// Variabelen voor het bijhouden van de laatste positie
float lastX = -1, lastY = -1;

unsigned long lastReconnectAttempt = 0;

// Status noodstop
bool emergencyStop = false;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 500;

// LED-updatefunctie
void updateLEDs(const String& movementDirection) {
  digitalWrite(LED_NORTH, (movementDirection == "NORTH"));
  digitalWrite(LED_EAST, (movementDirection == "EAST"));
  digitalWrite(LED_SOUTH, (movementDirection == "SOUTH"));
  digitalWrite(LED_WEST, (movementDirection == "WEST"));

  Serial.println("LED indicates direction: " + movementDirection);
}

// Functies om berichten via MQTT te versturen
void sendMessage(const char* message) {
  client.publish(commandTopic, message);
  Serial.println("Message sent: " + String(message));
}

void sendEmergencyStop() {
  String emergencyMsg = "{\"protocolVersion\":1.0,\"data\":{\"sender\":\"esp32\",\"target\":\"all\",\"msg\":\"EMERGENCY_STOP\"}}";
  sendMessage(emergencyMsg.c_str());
}

void sendResume() {
  String resumeSystemMsg = "{\"protocolVersion\":1.0,\"data\":{\"sender\":\"esp32\",\"target\":\"all\",\"msg\":\"RESUME\"}}";
  sendMessage(resumeSystemMsg.c_str());
}

// Callbackfunctie
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String incomingJSON = (char*)payload;

  Serial.println();
  Serial.println("Received MQTT message:");
  Serial.println(incomingJSON);

  // JSON met ArduinoJson
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, incomingJSON);
  if (error) {
    Serial.print("Error during JSON parsing: ");
    Serial.println(error.c_str());
    return;
  }

  // Controleren of de sender bot3 is
  const char* sender = doc["data"]["sender"];
  if (!sender || String(sender) != "bot3") {
    Serial.println("Incoming message is not addressed to bot3");
    return;
  }

  // X- en Y-coördinaten lezen
  float x = doc["data"]["msg"]["location"]["x"] | -1.0;
  float y = doc["data"]["msg"]["location"]["y"] | -1.0;
  if (x == -1.0 || y == -1.0) {
    Serial.println("Coordinates x/y not found in message");
    return;
  }

  // Print de positie van bot3
  Serial.printf("Current bot3 position: x=%.1f, y=%.1f\n", x, y);

  // Bepaal bewegingsrichting
  String heading = "STIL";
  float delta = 0.01;
  if (lastX >= 0 && lastY >= 0) {
    if (x - lastX > delta) heading = "EAST";
    else if (lastX - x > delta) heading = "WEST";
    else if (y - lastY > delta) heading = "NORTH";
    else if (lastY - y > delta) heading = "SOUTH";
  }

  lastX = x;
  lastY = y;

  // Update LED's alleen als er een richting is
  if (heading != "STIL") {
    Serial.println("Detected direction: " + heading);
    updateLEDs(heading);
  }
}

// Functie voor (her)verbinding met MQTT
bool reconnect() {
  char clientId[32];
  sprintf(clientId, "ESP32Client_%lu", millis());

  if (client.connect(clientId)) {
    Serial.println("MQTT connection successful!");
    client.subscribe(subscribeTopic);
    Serial.println("Subscribed to topic: " + String(subscribeTopic));

    // Verwerk direct de MQTT-queue
    client.loop();
    return true;
  } else {
    Serial.print("MQTT error code: ");
    Serial.println(client.state());
    return false;
  }
}

void setup() {
  Serial.begin(115200);

  // Stel LED-pinnen in als outputs
  pinMode(LED_NORTH, OUTPUT);
  pinMode(LED_EAST, OUTPUT);
  pinMode(LED_SOUTH, OUTPUT);
  pinMode(LED_WEST, OUTPUT);

  // Knop instellen met interne pull-up
  pinMode(BUTTON_STOP, INPUT_PULLUP);

  // Maak verbinding met WiFi
  Serial.print("Attempting to connect to WiFi: ");
  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nSuccessfully connected to WiFi!");

  // Stel de MQTT-server in
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  // Initialiseer de reconnect-timer
  lastReconnectAttempt = millis();
}

void loop() {
  // Check of er verbinding is. Zo niet, probeer opnieuw om de 5 seconden
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      Serial.println("Attempting to reconnect...");
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Verwerk binnenkomende berichten
    client.loop();
  }

  // Check status noodstopknop
  if (digitalRead(BUTTON_STOP) == LOW && (millis() - lastButtonPress > debounceDelay)) {
    lastButtonPress = millis();  // Tijdsregistratie voor debounce

    // Alleen activeren van noodstop, niet deactiveren
    if (!emergencyStop) {
      emergencyStop = true;
      Serial.println("Emergency stop activated.");
      sendEmergencyStop();
    }
    delay(200);
  }
}
