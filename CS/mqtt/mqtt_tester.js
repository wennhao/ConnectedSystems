const mqtt = require('mqtt');
const client = mqtt.connect('mqtt://test.mosquitto.org:1883');

client.on('connect', () => {
  console.log("MQTT tester verbonden");
  client.subscribe('robot/status', (err) => {
    if (err) console.error("Abonneren mislukt:", err);
  });
});

client.on('message', (topic, message) => {
  try {
    const data = JSON.parse(message.toString());
    console.log("Ontvangen bericht op topic", topic, ":", data);
  } catch (e) {
    console.error("Fout bij verwerken bericht:", e);
  }
});
