/** 
 * Connected Systems Dashboard - script.js
 * 
 * This script:
 * - Fetches robot data from the server
 * - Draws the grid and robots on canvas
 * - Sends commands (move, emergency stop)
 * - Validates input and output
 * - Displays command queues
 */

// Canvas and context
const canvas = document.getElementById('field');
const ctx = canvas.getContext('2d');
const gridSize = 10;
const cellSize = canvas.width / gridSize;

// Status elements
const systemStatusEl = document.getElementById('system-status');
const stopStatusEl = document.getElementById('stop-status');
const lastActionEl = document.getElementById('last-action');

// API endpoint base
const API_BASE = 'http://localhost:5001';

// Object to store robot data
let robots = {};

// Object to store robot queues
let robotQueues = {
    'bot1': [],
    'bot2': [],
    'bot3': []
};

// Emergency stop status
let emergencyActive = false;

// Obstacles in the field
const obstacles = [
    // Vertical obstacles column 1, 3, 6, 8
    {x: 1, y: 1}, {x: 1, y: 2}, {x: 1, y: 3},
    {x: 3, y: 1}, {x: 3, y: 2}, {x: 3, y: 3},
    {x: 6, y: 1}, {x: 6, y: 2}, {x: 6, y: 3},
    {x: 8, y: 1}, {x: 8, y: 2}, {x: 8, y: 3},

    // Vertical obstacles column 1, 3, 6, 8 (bottom half)
    {x: 1, y: 6}, {x: 1, y: 7}, {x: 1, y: 8},
    {x: 3, y: 6}, {x: 3, y: 7}, {x: 3, y: 8},
    {x: 6, y: 6}, {x: 6, y: 7}, {x: 6, y: 8},
    {x: 8, y: 6}, {x: 8, y: 7}, {x: 8, y: 8}
];

/** 
 * Logging to console with timestamp
 */
function log(level, message, data = null) {
    const timestamp = new Date().toISOString();
    const logMessage = `${timestamp} - ${level}: ${message}`;
    if (data) {
        console.log(logMessage, data);
    } else {
        console.log(logMessage);
    }
}

/**
 * Convert Webots coordinates to GUI grid positions
 * 
 * @param {number} x - Webots x-coordinate (0.0-0.9)
 * @param {number} y - Webots y-coordinate (0.0-0.9)
 * @returns {Object} - GUI x,y grid position (0-9)
 */
function coordinateToGridBlock(x, y) {
    return {
        // Webots y becomes GUI x
        x: Math.floor(y * 10),
        // Webots x becomes GUI y
        y: Math.floor(x * 10)
    };
}

/**
 * Check if a grid position contains an obstacle
 * 
 * @param {number} x - Grid x-position (0-9)
 * @param {number} y - Grid y-position (0-9)
 * @returns {boolean} - true if there is no obstacle
 */
function validateGridPosition(x, y) {
    // Within grid boundaries
    if (x < 0 || x >= gridSize || y < 0 || y >= gridSize) {
        return false;
    }

    // Check obstacles
    for (const obs of obstacles) {
        if (obs.x === x && obs.y === y) {
            return false;
        }
    }
    return true;
}

/**
 * Validate and correct coordinates
 * 
 * @param {number} x - Grid x-coordinate
 * @param {number} y - Grid y-coordinate
 * @returns {Object} - validated x,y coordinates
 */
function validateCoordinates(x, y) {
    // Make positive
    x = Math.abs(x);
    y = Math.abs(y);

    // Keep within grid
    x = Math.min(Math.max(Math.round(x), 0), gridSize-1);
    y = Math.min(Math.max(Math.round(y), 0), gridSize-1);

    return {x, y};
}

/**
 * Draw the grid and obstacles
 */
function drawGrid() {
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw grid lines
    ctx.strokeStyle = '#ccc';
    for (let i = 0; i <= gridSize; i++) {
        ctx.beginPath();
        ctx.moveTo(i * cellSize, 0);
        ctx.lineTo(i * cellSize, canvas.height);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(0, i * cellSize);
        ctx.lineTo(canvas.width, i * cellSize);
        ctx.stroke();
    }

    // Draw obstacles
    ctx.fillStyle = '#999';
    obstacles.forEach(obstacle => {
        ctx.fillRect(obstacle.x * cellSize, obstacle.y * cellSize, cellSize, cellSize);
    });
}

/**
 * Draw the robots on the grid
 */
function drawRobots() {
    // Color per robot
    const robotColors = {
        'bot1': '#3a7cec',  // blue
        'bot2': '#3cb371',  // green
        'bot3': '#e74c3c'   // red
    };

    // Loop through all robots
    for (const id in robots) {
        const robot = robots[id];
        if (robot && robot.msg && robot.msg.location) {
            // Get coordinates
            const { x, y } = robot.msg.location;

            // Convert coordinates for grid
            const gridPos = coordinateToGridBlock(x, y);

            // Draw robot as colored square
            const color = robotColors[id] || '#3a7cec';
            ctx.fillStyle = color;
            ctx.fillRect(gridPos.x * cellSize, gridPos.y * cellSize, cellSize, cellSize);

            // Draw ID as label
            ctx.fillStyle = 'white';
            ctx.font = '14px Arial';
            ctx.fillText(id, gridPos.x * cellSize + 5, gridPos.y * cellSize + 25);

            // Log transformation for debugging
            log('DEBUG', `Robot ${id}: Webots(${x}, ${y}) -> GUI(${gridPos.x}, ${gridPos.y})`);
        }
    }
}

/**
 * Update the entire field
 */
function updateField() {
    drawGrid();
    drawRobots();
}

/**
 * Fetch robot data from server
 */
async function fetchRobots() {
    try {
        const res = await fetch(`${API_BASE}/robots`);
        if (res.ok) {
            const data = await res.json();
            log('INFO', 'Received robot data:', data);
            // Update robots object with new data
            robots = data;
            // Update the field
            updateField();
        } else {
            log('ERROR', `Error fetching robot data: ${res.status} ${res.statusText}`);
            systemStatusEl.textContent = "connection error";
            systemStatusEl.style.color = "#ff5757";
        }
    } catch (err) {
        log('ERROR', 'Fetch error:', err);
        systemStatusEl.textContent = "offline";
        systemStatusEl.style.color = "#ff5757";
    }
}

/** 
 * Fetch queue information from server 
 */
async function fetchQueues() {
    try {
        const res = await fetch(`${API_BASE}/queues`);
        if (res.ok) {
            const data = await res.json();
            log('INFO', 'Received queue data:', data);
            robotQueues = data;
            // Update the queue display
            updateQueueDisplay();
        } else {
            log('ERROR', `Error fetching queues: ${res.status} ${res.statusText}`);
        }
    } catch (err) {
        log('ERROR', 'Queue fetch error:', err);
    }
}

/**
 * Fetch emergency stop status from server
 */
async function fetchEmergencyStatus() {
    try {
        const res = await fetch(`${API_BASE}/emergency_status`);
        if (res.ok) {
            const data = await res.json();
            emergencyActive = data.active;
            
            // Update UI
            if (emergencyActive) {
                stopStatusEl.textContent = "ACTIVE";
                stopStatusEl.classList.add('active');
                document.querySelector('.stop-btn').classList.add('active-stop');
            } else {
                stopStatusEl.textContent = "inactive";
                stopStatusEl.classList.remove('active');
                document.querySelector('.stop-btn').classList.remove('active-stop');
            }
        }
    } catch (err) {
        log('ERROR', 'Error fetching emergency stop status:', err);
    }
}

/**
 * Send emergency stop command
 */
function sendStop() {
    // Visual feedback
    document.querySelector('.stop-btn').classList.add('active-stop');
    
    fetch(`${API_BASE}/emergency_stop`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        log('INFO', 'Emergency stop sent, result:', data);
        
        // Update status
        stopStatusEl.textContent = "ACTIVE";
        stopStatusEl.classList.add('active');
        emergencyActive = true;
        
        // Update action log
        updateLastAction("emergency stop activated");
    })
    .catch(err => {
        log('ERROR', 'Error sending emergency stop:', err);
        alert('Error sending emergency stop: ' + err.message);
    });
}

/**
 * Resume after emergency stop
 */
function sendResume() {
    // Visual feedback
    document.querySelector('.stop-btn').classList.remove('active-stop');
    
    fetch(`${API_BASE}/resume`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        log('INFO', 'Resume sent, result:', data);
        
        // Update status
        stopStatusEl.textContent = "inactive";
        stopStatusEl.classList.remove('active');
        emergencyActive = false;
        
        // Update action log
        updateLastAction("system resumed");
    })
    .catch(err => {
        log('ERROR', 'Error sending resume:', err);
        alert('Error sending resume: ' + err.message);
    });
}

/**
 * Send movement command
 */
function sendMove() {
    // If emergency stop is active, don't allow movement
    if (emergencyActive) {
        alert('Cannot send move command during emergency stop.');
        return;
    }
    
    const unitId = document.getElementById('unitId').value;
    const targetStr = document.getElementById('target').value;
    const parts = targetStr.split(',');
    
    if (parts.length !== 2) {
        alert('Enter the target as x,y (e.g. 5,5)');
        return;
    }
    
    // Check if queue is not full
    if (robotQueues[unitId] && robotQueues[unitId].length >= 3) {
        alert(`Queue for ${unitId} is full (max 3 commands). Wait until there is space.`);
        return;
    }
    
    // Get coordinates and validate
    const guiX = Number(parts[0].trim());
    const guiY = Number(parts[1].trim());
    
    // Validate
    const validatedPos = validateCoordinates(guiX, guiY);
    
    // Check obstacles
    if (!validateGridPosition(validatedPos.x, validatedPos.y)) {
        alert('This location contains an obstacle or is out of bounds. Choose a different location.');
        return;
    }
    
    // Convert from GUI grid to Webots coordinates (note the swap and scale)
    const webotsX = validatedPos.y / 10; // GUI Y becomes Webots X
    const webotsY = validatedPos.x / 10; // GUI X becomes Webots Y
    
    const target = {
        x: webotsX,
        y: webotsY
    };
    
    log('INFO', `Move command: GUI(${validatedPos.x},${validatedPos.y}) -> Webots(${webotsX},${webotsY})`);
    
    fetch(`${API_BASE}/move`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            unitId,
            target
        })
    })
    .then(res => res.json())
    .then(data => {
        log('INFO', 'Move command sent, result:', data);
        
        if (data.status.includes("added to queue")) {
            updateLastAction(`${unitId} to (${validatedPos.x},${validatedPos.y}) added to queue (position ${data.queuePosition})`);
        } else {
            updateLastAction(`${unitId} to (${validatedPos.x},${validatedPos.y})`);
        }
        
        // Immediately fetch queues after adding for fresh data
        fetchQueues();
    })
    .catch(err => {
        log('ERROR', 'Error sending move command:', err);
        alert('Error sending move command: ' + err.message);
    });
}

/**
 * Clear the command queue for a robot
 */
function clearQueue() {
    // Get the selected robot ID
    const robotId = document.getElementById('unitId').value;
    
    // Confirm before clearing
    if (!confirm(`Are you sure you want to clear the command queue for ${robotId}?`)) {
        return;
    }
    
    fetch(`${API_BASE}/clear_queue`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ robotId })
    })
    .then(res => res.json())
    .then(data => {
        log('INFO', 'Queue cleared, result:', data);
        updateLastAction(`Queue cleared for ${robotId}`);
        
        // Immediately fetch queues for fresh data
        fetchQueues();
    })
    .catch(err => {
        log('ERROR', 'Error clearing queue:', err);
        alert('Error clearing queue: ' + err.message);
    });
}

/**
 * Update the queue display with robot queues
 */
function updateQueueDisplay() {
    const queueContent = document.getElementById('queue-content');
    if (!queueContent) return;
    
    queueContent.innerHTML = '';
    
    // Fixed set of robots to display
    const unitIds = ['bot1', 'bot2', 'bot3'];
    
    unitIds.forEach(id => {
        // Create robot queue container
        const robotQueue = document.createElement('div');
        robotQueue.className = 'robot-queue';
        
        // Determine current position
        let position = '-';
        let statusIcon = '<i class="fas fa-question-circle"></i>';
        let statusClass = 'disconnected';
        
        if (robots[id] && robots[id].msg && robots[id].msg.location) {
            const gridPos = coordinateToGridBlock(
                robots[id].msg.location.x, 
                robots[id].msg.location.y
            );
            position = `(${gridPos.x}, ${gridPos.y})`;
            statusIcon = '<i class="fas fa-check-circle"></i>';
            statusClass = 'connected';
        }
        
        // Create header for robot queue
        const header = document.createElement('div');
        header.className = 'robot-queue-header';
        header.innerHTML = `
            <h3><i class="fas fa-robot"></i> ${id}</h3>
            <div class="robot-status ${statusClass}">
                Position: <span class="position-value">${position}</span>
            </div>
        `;
        
        // Show queue for this robot
        const queueDiv = document.createElement('div');
        queueDiv.className = 'command-queue';
        
        if (robotQueues[id] && robotQueues[id].length > 0) {
            robotQueues[id].forEach((cmd, index) => {
                const targetGrid = coordinateToGridBlock(cmd.target.x, cmd.target.y);
                
                // Determine icon based on status
                let statusIcon = '';
                switch(cmd.status) {
                    case 'pending': 
                        statusIcon = '<i class="fas fa-clock"></i>'; 
                        break;
                    case 'active': 
                        statusIcon = '<i class="fas fa-play-circle"></i>'; 
                        break;
                    case 'completed': 
                        statusIcon = '<i class="fas fa-check-circle"></i>'; 
                        break;
                    case 'error': 
                        statusIcon = '<i class="fas fa-exclamation-circle"></i>'; 
                        break;
                }
                
                const cmdElement = document.createElement('div');
                cmdElement.className = `queue-item ${cmd.status}`;
                cmdElement.innerHTML = `
                    <div>
                        <span class="queue-pos">${index + 1}</span>
                        <span class="queue-target">To (${targetGrid.x}, ${targetGrid.y})</span>
                    </div>
                    <span class="queue-status ${cmd.status}">${statusIcon} ${cmd.status}</span>
                `;
                queueDiv.appendChild(cmdElement);
            });
        } else {
            queueDiv.innerHTML = '<p class="empty-queue"><i class="fas fa-inbox"></i> No commands in queue</p>';
        }
        
        // Assemble robot queue and add to container
        robotQueue.appendChild(header);
        robotQueue.appendChild(queueDiv);
        queueContent.appendChild(robotQueue);
    });
}

/**
 * Update the "last action" status
 */
function updateLastAction(action) {
    lastActionEl.textContent = action;
    
    // Show action for 5 seconds, then clear
    setTimeout(() => {
        if (lastActionEl.textContent === action) {
            lastActionEl.textContent = '';
        }
    }, 5000);
}

// Polling intervals for data retrieval
setInterval(fetchRobots, 1000);
setInterval(fetchQueues, 1000); // Also fetch queues
setInterval(fetchEmergencyStatus, 2000);

// Initial fetch
fetchRobots();
fetchQueues();
fetchEmergencyStatus();

// Init message
log('INFO', 'Dashboard initialized');
