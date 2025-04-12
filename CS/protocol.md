# Connected Systems Communication Protocol

## Table of contents
- [Introduction](#introduction)
- [Architecture Overview](#architecture-overview)
- [Message Specifications](#message-specifications)
- [Command Handling](#command-handling)
- [Collision Management](#collision-management)
- [System Interfaces](#system-interfaces)
- [Error Handling](#error-handling)
- [Security Considerations](#security-considerations)
- [Version Management](#version-management)

## Introduction
This protocol governs communication between robots, control servers, and user interfaces in the Connected Systems platform. It combines MQTT for real-time messaging and REST for dashboard interactions, enabling reliable coordination of autonomous robots in shared environments.

## Architecture overview

┌─────────────┐         MQTT       ┌────────────┐        REST      ┌────────────┐
│ Robots      │◄───────────────────│   Server   │◄─────────────────│ Dashboard  │
└─────────────┘                    └────────────┘                  └────────────┘


### Component Responsibilities
- **Robots**: Execute movement commands, detect obstacles, report status
- **Server**: Coordinate robot actions, manage command queues, process status updates
- **Dashboard**: Visualize robot positions, send commands, monitor system status

## Message Specifications

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

## Message Sequence Example
Dashboard -> Server: POST /api/command {target coordinates}
Server -> Robot: MQTT "robot/command" topic
Robot -> Server: MQTT "robot/status" updates
Server -> Dashboard: WebSocket updates

## Error handling workflow
1. Invalid command detection
2. Error message generation
3. Queue cleanup procedure
4. Recovery mechanism

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

## Command Handling

### Processing workflow
1. Command received via `robot/command` topic
2. Validation check (coordinates within 0.0-0.9 range)
3. Queue insertion (maximum 3 commands per robot)
4. Sequential execution with status verification

### Pseudo-code example
if current_position == target_position:
process_next_command()

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

## Collision Management

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

### Deadlock prevention
- Timeout-based resolution (5 seconds maximum wait)
- Priority inversion prevention
- Server-side monitoring of potential deadlocks

## System interfaces

### MQTT Endpoints
| Endpoint         | Direction       | QoS | Description                     | Frequency    |
|------------------|-----------------|-----|----------------------------------|--------------|
| robot/status     | Robots → Server | 1   | Continuous position updates     | 500ms        |
| robot/command    | Server → Robots | 2   | Critical control instructions   | On-demand    |

### QoS levels explained
- QoS 1 for status: Ensures delivery while minimizing overhead
- QoS 2 for commands: Guarantees exactly-once delivery for critical instructions

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

## Error handling

### Error workflow
1. **Detection**: Validate commands and states
2. **Notification**: Generate appropriate error responses
3. **Recovery**: Implement fallback procedures
4. **Logging**: Record errors for troubleshooting

### Common error types
| Error Code           | Description                       | Recovery Action                  |
|----------------------|-----------------------------------|----------------------------------|
| INVALID_COORDINATES  | Coordinates out of range          | Use closest valid coordinates    |
| QUEUE_FULL           | Command queue at capacity         | Wait and retry                   |
| PATH_BLOCKED         | No valid path to destination      | Find alternative route           |
| COMMUNICATION_ERROR  | MQTT connection failure           | Implement automatic reconnection |

## Security considerations

### Communication security
- MQTT broker authentication (not implemented in test environment)
- TLS encryption recommended for production
- Input validation on all endpoints

### Access control
- Dashboard authentication for production environments
- Command validation before execution
- Rate limiting for API endpoints

## Version management

### Protocol versioning
- Current version: 1.0
- Version field required in all messages
- Backward compatibility maintained for minor version changes
- Major version changes may require system updates
