# Connected Systems Communication Protocol

## Table of contents
- [Introduction](#introduction)
- [Message specifications](#message-specifications)
- [Command handling](#command-handling)
- [Collision management](#collision-management)
- [System interfaces](#system-interfaces)


## Introduction
This document is used to agree upon how the robots and their digital twins in webots communicate with eachother

## Message specifications

### Core structure
{
"protocolVersion": 1.0,
"data": {
"sender": "bot1",
"target": "server",
"msg": {}
}
}

### Status update example
{
"protocolVersion": 1.0,
"data": {
"sender": "bot1",
"target": "server",
"msg": {
"location": {"x": 0.3, "y": 0.5},
"obstacles": ["north"],
"emergency": false
}
}
}

## Message sequence example
Dashboard -> Server: POST /api/command {target coordinates}
Server -> Robot: MQTT "robot/command" topic
Robot -> Server: MQTT "robot/status" updates
Server -> Dashboard: WebSocket updates

### Command types

| Command type     | Format example                                                | Purpose                          |
|------------------|--------------------------------------------------------------|----------------------------------|
| Movement         | `{"command":"MOVE","target":{"x":0.5,"y":0.7}}`              | Navigate to specified coordinates|
| Emergency stop   | `{"command":"EMERGENCY_STOP"}`                                | Immediate system halt            |
| Resume           | `{"command":"RESUME"}`                                        | Resume after emergency stop      |
| Queue clear      | `{"command":"CLEAR_QUEUE"}`                                   | Reset pending commands           |

## Message sequence diagrams

### Normal operation flow
Dashboard → Server: POST /move {robotId, target coordinates}
Server → Robot: MQTT "robot/command" topic (MOVE command)
Robot → Server: MQTT "robot/status" updates (position)
Server → Dashboard: Status updates via polling

### Emergency sequence
Dashboard/ESP32 → Server: POST /emergency_stop
Server → All Robots: MQTT "robot/command" topic (EMERGENCY_STOP)
Robots → Server: MQTT "robot/status" (emergency:true)
Server → Dashboard: Emergency status via polling

## Command handling

### Processing workflow
1. Command received via `robot/command` topic
2. Validation check (coordinates within 0.0-0.9 range)
3. Queue insertion (maximum 3 commands per robot)
4. Sequential execution with status verification

### Priority system
- First-In-First-Out (FIFO) processing
- Emergency commands bypass queue
- Robot ID-based priority (lower IDs first)
- Deadlock prevention through timeout mechanisms

### Queue management
- Maximum queue size: 3 commands per robot
- Commands can be cleared via dashboard
- Queue status visible in dashboard
- Command states: pending, active, completed, error

## Collision management

### Avoidance strategy
1. **Position sharing**: Broadcast updates every 500ms
2. **Path prediction**: Calculate future positions using:
predicted_x = current_x + (velocity_x * time_delta)
predicted_y = current_y + (velocity_y * time_delta)

3. **Dynamic routing**: Recalculate paths when obstacles detected
4. **Safety buffer**: 0.2 unit clearance around all objects

### Resolution process
1. Detect potential collision through position analysis
2. Higher-priority robot maintains course
3. Lower-priority robot recalculates path
4. Update new route through modified Dijkstra's algorithm
5. If no path found, wait until higher-priority robot passes

## System interfaces

### MQTT Endpoints
| Endpoint         | Direction       | Description                     | Frequency    |
|------------------|-----------------|---------------------------------|--------------|
| robot/status     | Robots → Server | Continuous position updates     | 500ms        |
| robot/command    | Server → Robots | Critical control instructions   | On-demand    |

### REST API Endpoints

#### Move command
POST /move
Content-Type: application/json

{
"unitId": "bot1",
"target": {"x": 0.7, "y": 0.4}
}

#### Emergency stop
POST /emergency_stop

#### Resume operation
POST /resume

#### Clear ueue
POST /clear_queue
Content-Type: application/json

{
"robotId": "bot1"
}

### Error responses
{
"error": "INVALID_COORDINATES",
"message": "X value exceeds maximum allowed range"
}

### Communication security
- MQTT broker authentication (not implemented in test environment)
- TLS encryption recommended for production
- Input validation on all endpoints
