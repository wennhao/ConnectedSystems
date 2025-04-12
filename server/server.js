/**
 * Connected Systems Node.js Server
 *
 * Deze server:
 * - Verbindt met MQTT broker en abonneert op robot/status
 * - Biedt REST endpoints voor dashboard communicatie
 * - Stuurt commando's naar robots via MQTT robot/command
 * - Houdt robotstatussen bij in memory
 * - Beheert wachtrijen voor robotcommando's
 */

const express = require('express');
const mqtt = require('mqtt');
const cors = require('cors');
const bodyParser = require('body-parser');

// Express app setup
const app = express();
app.use(cors());
app.use(bodyParser.json());

// In-memory opslag van robotstatussen
let robotData = {};

// In-memory opslag van robotwachtrijen
let robotQueues = {
    'bot1': [],
    'bot2': [],
    'bot3': []
};
const MAX_QUEUE_SIZE = 3;

// Noodstop status bijhouden
let emergencyStopActive = false;

// MQTT Instellingen
const MQTT_BROKER = 'mqtt://test.mosquitto.org:1883';
const MQTT_TOPICS = {
    STATUS: 'robot/status',
    COMMAND: 'robot/command'
};

// Logging helper
function log(type, message, data = null) {
    const timestamp = new Date().toISOString();
    const logMessage = `${timestamp} - ${type}: ${message}`;
    if (data) {
        console.log(logMessage, data);
    } else {
        console.log(logMessage);
    }
}

// Verbinding maken met de publieke MQTT-broker
log('INFO', `Verbinden met MQTT broker ${MQTT_BROKER}...`);
const client = mqtt.connect(MQTT_BROKER);

client.on('connect', () => {
    log('INFO', `Verbonden met MQTT broker ${MQTT_BROKER}`);
    // Abonneren op robot status topic
    client.subscribe(MQTT_TOPICS.STATUS, (err) => {
        if (err) {
            log('ERROR', `Fout bij abonneren op ${MQTT_TOPICS.STATUS}:`, err);
        } else {
            log('INFO', `Geabonneerd op topic: ${MQTT_TOPICS.STATUS}`);
        }
    });
});

client.on('error', (err) => {
    log('ERROR', 'MQTT verbindingsfout:', err);
});

// Functie om te controleren of een robot zijn doel heeft bereikt
function hasRobotReachedTarget(robotId, target) {
    if (!robotData[robotId] || !robotData[robotId].msg || !robotData[robotId].msg.location) {
        return false;
    }
    
    const location = robotData[robotId].msg.location;
    // We gebruiken een kleine marge voor floating point vergelijking
    const EPSILON = 0.05;
    
    return Math.abs(location.x - target.x) < EPSILON && 
           Math.abs(location.y - target.y) < EPSILON;
}

// Functie om de wachtrij te verwerken en de volgende opdracht te sturen
function processQueue(robotId) {
    if (robotQueues[robotId].length === 0) {
        return; // Geen opdrachten in de wachtrij
    }
    
    if (emergencyStopActive) {
        log('WARNING', `Can't process queue for ${robotId}: emergency stop active`);
        return;
    }
    
    // Als er een actieve opdracht is, controleer of deze is voltooid
    const currentCommand = robotQueues[robotId][0];
    if (currentCommand.status === 'active') {
        // Controleer of robot doel heeft bereikt
        if (hasRobotReachedTarget(robotId, currentCommand.target)) {
            // Markeer als voltooid en verwijder uit de wachtrij
            currentCommand.status = 'completed';
            robotQueues[robotId].shift();
            log('INFO', `Robot ${robotId} has reached target: (${currentCommand.target.x}, ${currentCommand.target.y})`);
            
            // Verwerk volgende opdracht in wachtrij (recursief)
            processQueue(robotId);
        }
        return;
    }
    
    // Stuur de eerste opdracht in de wachtrij
    const nextCommand = robotQueues[robotId][0];
    nextCommand.status = 'active';
    
    const command = {
        protocolVersion: 1.0,
        data: {
            sender: "server",
            target: robotId,
            msg: {
                command: "MOVE",
                target: nextCommand.target
            }
        }
    };
    
    publishCommand(MQTT_TOPICS.COMMAND, command, (err) => {
        if (err) {
            log('ERROR', `Error sending move command to ${robotId}:`, err);
            nextCommand.status = 'error';
        } else {
            log('INFO', `Move command sent to ${robotId}: (${nextCommand.target.x}, ${nextCommand.target.y})`);
        }
    });
}

client.on('message', (topic, message) => {
    try {
        // Bericht parsen
        const data = JSON.parse(message.toString());
        // Controleren of bericht correct formaat heeft
        if (!data.data || !data.data.sender) {
            log('WARNING', 'Invalid MQTT message format:', data);
            return;
        }

        const sender = data.data.sender;
        // Status opslaan
        robotData[sender] = data.data;
        
        // Controleer of huidige opdracht in queue is voltooid
        if (robotQueues[sender] && robotQueues[sender].length > 0) {
            processQueue(sender);
        }
        
        // Detail logging alleen bij belangrijke wijzigingen
        const location = data.data.msg && data.data.msg.location ?
            `(${data.data.msg.location.x}, ${data.data.msg.location.y})` : 'unknown';
        log('INFO', `Status received from ${sender}: position=${location}`);
    } catch (error) {
        log('ERROR', "Error processing MQTT message:", error);
    }
});

// REST API Endpoints

// GET /robots - Haal alle robotstatussen op
app.get('/robots', (req, res) => {
    res.json(robotData);
});

// GET /queues - Haal de huidige commandowachtrijen op
app.get('/queues', (req, res) => {
    res.json(robotQueues);
});

// GET /emergency_status - Haal huidige noodstop status op 
// TODO: Error handling verbeteren
app.get('/emergency_status', (req, res) => {
    res.json({ active: emergencyStopActive });
});

// POST /emergency_stop - Activeer noodstop
app.post('/emergency_stop', (req, res) => {
    log('WARNING', "EMERGENCY STOP command received");
    emergencyStopActive = true;
    
    const command = {
        protocolVersion: 1.0,
        data: {
            sender: "server",
            target: "all",
            msg: "EMERGENCY_STOP"
        }
    };

    // Verstuur 3x voor zekerheid
    for (let i = 0; i < 3; i++) {
        // Eerste implementatie zonder retries
        publishCommand(MQTT_TOPICS.COMMAND, command, (err) => {
            if (err) {
                log('ERROR', `Attempt ${i+1}: Error sending emergency stop:`, err);
            } else {
                log('INFO', `Attempt ${i+1}: EMERGENCY STOP command sent`);
            }
        });
    }

    res.json({ status: "Emergency stop activated", active: true }); // Onzekere response
});

// POST /resume - Deactiveer noodstop
app.post('/resume', (req, res) => {
    log('INFO', "RESUME command received");
    emergencyStopActive = false;
    
    const command = {
        protocolVersion: 1.0,
        data: {
            sender: "server",
            target: "all",
            msg: "RESUME"
        }
    };

    publishCommand(MQTT_TOPICS.COMMAND, command, (err) => {
        if (err) {
            log('ERROR', "Error sending resume command:", err);
            res.status(500).json({ status: "Error sending resume command" });
        } else {
            log('INFO', "RESUME command sent via MQTT");
            res.json({ status: "Resume command sent", active: false });
        }
    });
});

// POST /move - Verstuur bewegingscommando
app.post('/move', (req, res) => {
    // Controleer of er een noodstop actief is
    if (emergencyStopActive) {
        log('WARNING', "Move command rejected: emergency stop active");
        return res.status(403).json({
            status: "Move command rejected: emergency stop is active"
        });
    }

    // Haal parameters uit request
    const { unitId, target } = req.body;

    // Valideer parameters
    if (!unitId || !target || typeof target.x !== 'number' || typeof target.y !== 'number') {
        log('WARNING', "Invalid parameters for move command:", req.body);
        return res.status(400).json({ status: "Invalid parameters" });
    }

    // Coördinaten valideren en positief maken
    const validTarget = {
        x: Math.abs(parseFloat(target.x.toFixed(1))),
        y: Math.abs(parseFloat(target.y.toFixed(1)))
    };
    
    // Controleer of queue bestaat en initialiseer indien nodig
    if (!robotQueues[unitId]) {
        robotQueues[unitId] = [];
    }
    
    // Controleer of wachtrij niet vol is
    if (robotQueues[unitId].length >= MAX_QUEUE_SIZE) {
        log('WARNING', `Queue full for ${unitId}, command rejected`);
        return res.status(429).json({ 
            status: "Queue full", 
            message: `Maximum queue size (${MAX_QUEUE_SIZE}) reached for ${unitId}`
        });
    }
    
    // Voeg commando toe aan wachtrij
    const newCommand = {
        target: validTarget,
        status: 'pending',
        timestamp: Date.now()
    };
    
    robotQueues[unitId].push(newCommand);
    log('INFO', `Move command added to queue for ${unitId}: (${validTarget.x}, ${validTarget.y})`);
    
    // Als dit het enige commando in de wachtrij is, verwerk het meteen
    if (robotQueues[unitId].length === 1) {
        processQueue(unitId);
    }
    
    res.json({
        status: `Move command added to queue for ${unitId}`,
        target: validTarget,
        queuePosition: robotQueues[unitId].length
    });
});

// POST /clear_queue - Clear the command queue for a robot
app.post('/clear_queue', (req, res) => {
    const { robotId } = req.body;
    
    // Validate robot ID
    if (!robotId || !robotQueues[robotId]) {
        log('WARNING', "Invalid robot ID for queue clearing:", robotId);
        return res.status(400).json({ status: "Invalid robot ID" });
    }
    
    // Clear the queue
    robotQueues[robotId] = [];
    log('INFO', `Queue cleared for ${robotId}`);
    
    res.json({
        status: `Queue cleared for ${robotId}`,
        queue: robotQueues[robotId]
    });
});

// Helper functie voor MQTT publiceren met errorhandling
function publishCommand(topic, command, callback) {
    const payload = JSON.stringify(command);
    client.publish(topic, payload, (err) => {
        if (err) {
            log('ERROR', `Error publishing to ${topic}:`, err);
            callback(err);
        } else {
            log('DEBUG', `Message published to ${topic}:`, payload);
            callback(null);
        }
    });
}

// Regelmatig de wachtrijen verwerken
setInterval(() => {
    if (!emergencyStopActive) {
        Object.keys(robotQueues).forEach(robotId => {
            processQueue(robotId);
        });
    }
}, 1000); // Every second

// Start de server op poort 5001
const PORT = 5001;
app.listen(PORT, () => {
    log('INFO', `Node.js server running on port ${PORT}`);
});

// shutdown
process.on('SIGINT', () => {
    log('INFO', 'Server shutting down...');
    if (client) {
        client.end();
    }
    process.exit();
});
