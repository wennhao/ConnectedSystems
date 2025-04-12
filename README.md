# Connected Systems - Robot coordination platform

A distributed system for coordinating autonomous robots with real-time communication, collision avoidance, and web-based control.

## Key features
- Real-time MQTT communication between robots/server
- REST API for dashboard interactions
- Dynamic pathfinding with obstacle/collision avoidance
- Priority-based command queuing system
- Emergency stop/resume functionality
- Web-based monitoring dashboard

## Technologies
- **Core protocol**: MQTT + REST
- **Robots**: Webots simulator + Python controllers
- **Server**: Node.js + Express
- **Dashboard**: HTML5/Canvas + JavaScript + CSS
- **Hardware**: ESP32 microcontrollers
- **Containerization**: Docker

## Installation
1. git clone https://github.com/ChevanR/Connected-Systems.git
2. cd connected-systems
3. docker-compose up --build


## System structure
- ├── robots/
- │ └── controllers/
- │ └── basic_controller.py
- ├── server/
- │ ├── server.js
- │ └── Dockerfile.server
- ├── dashboard/
- │ ├── public/
- │ │ ├── index.html
- │ │ ├── style.css
- │ │ └── script.js
- │ └── Dockerfile.dashboard
- ├── protocol.md
- └── docker-compose.yml


## Usage
1. Start system: `docker-compose up`
2. Access dashboard: `http://localhost:8080`
3. Send commands via:
   - Web interface
   - REST API: `POST /api/command`
   - MQTT: `robot/command` topic

## API Documentation
See [protocol.md](./protocol.md) for detailed message specifications and endpoints.

## Troubleshooting
Monitor MQTT traffic:
mosquitto_sub -h test.mosquitto.org -t "robot/#" -v

## Contributors
- Chevan ([@chevanr](https://github.com/chevanr))
- Wen ([@wennhao](https://github.com/wennhao))
- Jason ([@jason](https://github.com/jason))
