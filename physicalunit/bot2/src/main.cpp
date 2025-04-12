#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WIFI
const char* ssid = "Jamal";
const char* password = "abc12345";

// MQTT
const char* mqtt_server   = "test.mosquitto.org";
const int   mqtt_port     = 1883;
const char* topic_subscribe = "robot/status";
const char* topic_command   = "robot/command";

// define pins
#define LED_NORTH 25 //red
#define LED_EAST  33 //blue
#define LED_SOUTH 26 //yellow
#define LED_WEST  27 //green
#define BUTTON_STOP 16

WiFiClient espClient;
PubSubClient client(espClient);

// save last x and y coordinates
float lastX = -1, lastY = -1;

unsigned long lastReconnectAttempt = 0;

// variables for emergency stop
bool emergencyActive = false; 
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 500;

// led helper functions
void updateLEDs(const String &richting) {
  digitalWrite(LED_NORTH, (richting == "NORTH"));
  digitalWrite(LED_EAST,  (richting == "EAST"));
  digitalWrite(LED_SOUTH, (richting == "SOUTH"));
  digitalWrite(LED_WEST,  (richting == "WEST"));
  
  Serial.println("LED richting: " + richting);
}

// publish helper functions
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

// callback function when a message is received
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String jsonString = (char*)payload;
  
  Serial.println();
  Serial.println("MQTT-bericht ontvangen:");
  Serial.println(jsonString);
  
  // json parsing with ArduinoJson
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.print("Fout bij JSON-parsen: ");
    Serial.println(error.c_str());
    return;
  }
  
  // check if the message is for bot2
  const char* sender = doc["data"]["sender"];
  if (!sender || String(sender) != "bot2") {
    Serial.println("Bericht is niet voor bot2");
    return;
  }
  
  // read the x and y coordinates
  float x = doc["data"]["msg"]["location"]["x"] | -1.0;
  float y = doc["data"]["msg"]["location"]["y"] | -1.0;
  if (x == -1.0 || y == -1.0) {
    Serial.println("x/y niet gevonden in bericht");
    return;
  }
  
  // show the coordinates
  Serial.printf("Positie bot2: x=%.1f, y=%.1f\n", x, y);
  
  // calculate the direction
  String direction = "STIL";
  float delta = 0.01;
  if (lastX >= 0 && lastY >= 0) {
    if      (x - lastX > delta)  direction = "EAST";
    else if (lastX - x > delta)   direction = "WEST";
    else if (y - lastY > delta)   direction = "NORTH";
    else if (lastY - y > delta)   direction = "SOUTH";
  }
  
  // update last coordinates
  lastX = x;
  lastY = y;
  
  // update LEDs
  if (direction != "STIL") {
    Serial.println("direction: " + direction);
    updateLEDs(direction);
  }
}

// reconnect function
// try to reconnect to MQTT broker
// return true if successful, false otherwise
// this function is called in the loop
bool reconnect() {
  char clientId[32];
  sprintf(clientId, "ESP32Client_%lu", millis());
  
  if (client.connect(clientId)) {
    Serial.println("Verbonden met MQTT!");
    client.subscribe(topic_subscribe);
    Serial.println("Geabonneerd op: " + String(topic_subscribe));
    
    client.loop();
    return true;
  } else {
    Serial.print("MQTT-foutcode: ");
    Serial.println(client.state());
    return false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_NORTH, OUTPUT);
  pinMode(LED_EAST, OUTPUT);
  pinMode(LED_SOUTH, OUTPUT);
  pinMode(LED_WEST, OUTPUT);
  
  // Set up button with internal pull-up resistor
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  
  // connect to WiFi
  Serial.print("Verbinden met WiFi ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi verbonden!");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  lastReconnectAttempt = millis();
}

void loop() {
  // check if the WiFi is connected, if not, try to reconnect in 5 seconds
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
    // process MQTT messages
    client.loop();
  }
  
  // check if the emergency stop button is pressed
  if (digitalRead(BUTTON_STOP) == LOW && (millis() - lastButtonPress > debounceDelay)) {
    lastButtonPress = millis(); // update last button press time
    
    if (!emergencyActive) {
      emergencyActive = true;
      Serial.println("Noodstop ingeschakeld. Systeem pauzeren.");
      publishEmergencyStop();
    } else {
      emergencyActive = false;
      Serial.println("Noodstop uitgeschakeld. Systeem hervatten.");
      publishResume();
    }
    
    // wait for the button to be released
    delay(200);
  }
}
