#include <WiFi.h> 
#include <PubSubClient.h>
#include <ArduinoJson.h>

//  WiFi 
// Werkt soms niet bij herstart
const char* ssid = "iPhone";
const char* password = "12345678"; // TODO: Wachtwoord veiliger maken
 
//  MQTT 
const char* mqtt_server   = "test.mosquitto.org";
const int   mqtt_port     = 1883;
const char* topic_subscribe = "robot/status";
const char* topic_command   = "robot/command"; // Noodstop-/Resume-berichten

//  LED-pinnen 
#define LED_NORTH 25
#define LED_EAST  26
#define LED_SOUTH 27
#define LED_WEST  14

// Knop (noodstop)
#define BUTTON_STOP 33

WiFiClient espClient;
PubSubClient client(espClient);

// Positie bijhouden om richting te bepalen
float lastX = -1, lastY = -1;

// Timer voor reconnect
unsigned long lastReconnectAttempt = 0;

// Variabelen voor de noodstop-status en knop-debounce
bool emergencyActive = false;  // false = systeem actief, true = noodstop
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 500; // 500 ms debounce-tijd

// LED-hulpfunctie
void updateLEDs(const String &richting) {
  digitalWrite(LED_NORTH, (richting == "NORTH"));
  digitalWrite(LED_EAST,  (richting == "EAST"));
  digitalWrite(LED_SOUTH, (richting == "SOUTH"));
  digitalWrite(LED_WEST,  (richting == "WEST"));
  
  Serial.println("LED richting: " + richting);
}

// Functies om MQTT-berichten te publiceren
void publishMessage(const char* message) {
  client.publish(topic_command, message);
  Serial.println("Bericht verzonden: " + String(message));
}

void publishEmergencyStop() {
  String stopMsg = "{\"protocolVersion\":1.0,\"data\":{\"sender\":\"esp32\",\"target\":\"all\",\"msg\":\"EMERGENCY_STOP\"}}";
  publishMessage(stopMsg.c_str());
}

void publishResume() {
  String resumeMsg = "{\"protocolVersion\":1.0,\"data\":{\"sender\":\"esp32\",\"target\":\"all\",\"msg\":\"RESUME\"}}";
  publishMessage(resumeMsg.c_str());
}

// Callback: wordt aangeroepen als er een MQTT-bericht binnenkomt
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String jsonString = (char*)payload;
  
  Serial.println();
  Serial.println("MQTT-bericht ontvangen:");
  Serial.println(jsonString);
  
  // JSON parsen met ArduinoJson
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.print("Fout bij JSON-parsen: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Controleren of het bericht van bot1 komt
  const char* sender = doc["data"]["sender"];
  if (!sender || String(sender) != "bot1") {
    Serial.println("Bericht is niet voor bot1");
    return;
  }
  
  // x- en y-coördinaten uitlezen
  float x = doc["data"]["msg"]["location"]["x"] | -1.0;
  float y = doc["data"]["msg"]["location"]["y"] | -1.0;
  if (x == -1.0 || y == -1.0) {
    Serial.println("x/y niet gevonden in bericht");
    return;
  }
  
  // Positie tonen
  Serial.printf("Positie bot1: x=%.1f, y=%.1f\n", x, y);
  
  // Richting bepalen op basis van delta-verschillen
  String richting = "STIL";
  float delta = 0.01;
  if (lastX >= 0 && lastY >= 0) {
    if      (x - lastX > delta)  richting = "EAST";
    else if (lastX - x > delta)   richting = "WEST";
    else if (y - lastY > delta)   richting = "NORTH";
    else if (lastY - y > delta)   richting = "SOUTH";
  }
  
  // Huidige positie opslaan
  lastX = x;
  lastY = y;
  
  // Alleen LED's updaten bij echte richtingverandering (niet STIL)
  if (richting != "STIL") {
    Serial.println("Richting: " + richting);
    updateLEDs(richting);
  }
}

//  Reconnect met een unieke clientId om conflicten te vermijden 
bool reconnect() {
  char clientId[32];
  sprintf(clientId, "ESP32Client_%lu", millis());
  
  if (client.connect(clientId)) {
    Serial.println("Verbonden met MQTT!");
    client.subscribe(topic_subscribe);
    Serial.println("Geabonneerd op: " + String(topic_subscribe));
    
    // Direct verwerken
    client.loop();
    return true;
  } else {
    Serial.print("MQTT-foutcode: ");
    Serial.println(client.state());
    return false;
  }
}

//  Setup 
void setup() {
  // Eerste opstartpoging
  Serial.begin(115200);
  
  // LED-pinnen als OUTPUT instellen
  pinMode(LED_NORTH, OUTPUT);
  pinMode(LED_EAST, OUTPUT);
  pinMode(LED_SOUTH, OUTPUT);
  pinMode(LED_WEST, OUTPUT);
  
  // Knop instellen met interne pull-up
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  
  // Verbinden met WiFi
  Serial.print("Verbinden met WiFi ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Laadanimatie
  }
  Serial.println("\nWiFi verbonden!");
  
  // MQTT-client instellen
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Start reconnect-timer
  lastReconnectAttempt = millis();
}

//  Loop 
void loop() {
  // Controleren of we verbonden zijn met MQTT; zo niet, reconnect elke 5 seconden
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      Serial.println("Proberen te reconnecten...");
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Verwerk inkomende MQTT-berichten
    client.loop();
  }
  
  // Controleer of de noodstop-knop is ingedrukt met debounce
  if (digitalRead(BUTTON_STOP) == LOW && (millis() - lastButtonPress > debounceDelay)) {
    lastButtonPress = millis(); // Tijdstip bijwerken voor debounce
    
    if (!emergencyActive) {
      emergencyActive = true;
      Serial.println("Noodstop ingeschakeld. Systeem pauzeren.");
      publishEmergencyStop();
    } else {
      emergencyActive = false;
      Serial.println("Noodstop uitgeschakeld. Systeem hervatten.");
      publishResume();
    }
    
    // Extra kleine delay om herhaald indrukken te voorkomen
    delay(200);
  }
}
