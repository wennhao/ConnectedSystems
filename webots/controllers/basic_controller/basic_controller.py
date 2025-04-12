"""
Webots robot controller voor Connected Systems.

Deze controller:
- Beweegt de robot naar de doelpositie
- Detecteert obstakels met sensoren
- Communiceert via MQTT met de server
- Handelt MOVE en EMERGENCY_STOP commando's af
- Vermijdt botsingen met andere robots
"""

import paho.mqtt.client as mqtt
import json
import time
import random
import logging
import sys
import heapq
from controller import Supervisor  # type: ignore

#  Logging configuratie 
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(), logging.FileHandler("robot_controller.log")]
)
logger = logging.getLogger("RobotController")

#  Webots initialisatie 
try:
    robot = Supervisor()
    supervisorNode = robot.getSelf()
    timestep = int(robot.getBasicTimeStep())
    logger.info("Webots robot succesvol geïnitialiseerd")
except Exception as e:
    logger.error("Fout bij initialisatie van Webots robot: %s", e)
    sys.exit(1)

# Configuratie
STEP_SIZE = 0.1 
OBSTACLE_THRESHOLD = 400  
MIN_BOUND, MAX_BOUND = 0.0, 0.9
START_POS = [0.0, 0.0, 0.0]  # verander dit voor elke bot
ROBOT_SAFETY_MARGIN = 2  
PREDICTION_STEPS = 3 
ROBOT_PROXIMITY_THRESHOLD = 0.25

#  Gridlayout (1 = pad, 0 = muur) 
GRID = [
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1]
]
GRID_HEIGHT = len(GRID)
GRID_WIDTH = len(GRID[0])

# Willekeurige startdoelpositie binnen grenzen
TARGET_POS = [
    round(random.uniform(0.3, 0.7), 1),
    round(random.uniform(0.3, 0.7), 1)
]

# Bijhouden laatste doelpositie voor noodstop herstel
LAST_TARGET_POS = None

# Globale variabele voor noodstop
emergency_stop = False

# Dictionary om informatie over andere robots bij te houden
other_robots = {}

# Robot ID
ROBOT_ID = "bot1"  # Verander dit naar "bot1", "bot2", of "bot3" voor verschillende robots

logger.info("Configuratie: START_POS=%s, TARGET_POS=%s, ROBOT_ID=%s", START_POS, TARGET_POS, ROBOT_ID)

#  Positie en rotatie instellen 
try:
    trans = supervisorNode.getField("translation")
    rot = supervisorNode.getField("rotation")
    trans.setSFVec3f(START_POS)
    rot.setSFRotation([0, 0, 1, 0])
    logger.info("Positie en rotatie succesvol ingesteld")
except Exception as e:
    logger.error("Fout bij instellen positie/rotatie: %s", e)
    sys.exit(1)

#  MQTT instellingen 
BROKER = "test.mosquitto.org"
PORT = 1883
TOPIC_PUBLISH = "robot/status"
TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"  # Zelfde als publish topic

#  Sensoren en LEDs initialiseren 
try:
    # Sensoren
    sensor_N = robot.getDevice("DS_N")
    sensor_E = robot.getDevice("DS_E")
    sensor_S = robot.getDevice("DS_S")
    sensor_W = robot.getDevice("DS_W")
    sensor_N.enable(timestep)
    sensor_E.enable(timestep)
    sensor_S.enable(timestep)
    sensor_W.enable(timestep)
    
    # LEDs
    led_N = robot.getDevice("RED")
    led_E = robot.getDevice("BLUE")
    led_S = robot.getDevice("YELLOW")
    led_W = robot.getDevice("GREEN")
    
    logger.info("Sensoren en LED's succesvol geïnitialiseerd")
except Exception as e:
    logger.error("Fout bij initialisatie sensoren/LED's: %s", e)
    sys.exit(1)

#  Validatiefuncties 
def validate_coordinates(x, y):
    # Coordinaten nooit negatief
    x = abs(x)
    y = abs(y)
    
    # Binnen grenzen houden
    x = min(max(x, MIN_BOUND), MAX_BOUND)
    y = min(max(y, MIN_BOUND), MAX_BOUND)
    
    # Afronden op 1 decimaal voor consistente stappen
    x = round(x, 1)
    y = round(y, 1)
    
    logger.debug("Coordinaten gevalideerd: (%f, %f) -> (%f, %f)", x, y, x, y)
    return x, y

#  MQTT verbinding opzetten 
mqtt_connected = False
try:
    client = mqtt.Client(client_id=f"WebotsRobot_{ROBOT_ID}_{int(time.time())}", protocol=mqtt.MQTTv311)
    client.connect(BROKER, PORT)
    mqtt_connected = True
    logger.info("Verbonden met MQTT-broker %s:%d", BROKER, PORT)
except Exception as e:
    logger.error("Fout bij verbinden met MQTT-broker: %s", e)

#  Voorrangssysteem voor botsingsvermijding 
def should_yield_to_robot(my_id, other_id):
    return my_id > other_id

#  MQTT statusverwerking functie 
def on_status(client, userdata, msg):
   # Verwerk inkomende MQTT statusberichten van andere robots
    global other_robots
    
    try:
        payload = msg.payload.decode()
        status_data = json.loads(payload)
        
        if "data" in status_data:
            # Sla positie van andere robot op
            robot_id = status_data["data"].get("sender")
            if robot_id == ROBOT_ID:  # Sla eigen berichten over
                return
                
            location = status_data["data"]["msg"].get("location")
            if location:
                other_robots[robot_id] = {
                    "x": location["x"],
                    "y": location["y"],
                    "timestamp": time.time()
                }
                logger.debug(f"Positie van {robot_id} bijgewerkt: ({location['x']}, {location['y']})")
    except Exception as e:
        logger.error(f"Fout bij verwerken robotstatus: {e}")

#  MQTT commando verwerking functie 
def on_command(client, userdata, msg):
    global TARGET_POS, emergency_stop, LAST_TARGET_POS, path_cache
    
    try:
        logger.info("MQTT bericht ontvangen op %s", msg.topic)
        # Bericht decoderen en parsen
        payload = msg.payload.decode()
        logger.debug("Ontvangen payload: %s", payload)
        command_data = json.loads(payload)
        logger.info("MQTT commando ontvangen: %s", command_data)
        
        if "data" in command_data:
            # Controleer of dit commando voor deze robot is bedoeld
            target = command_data["data"].get("target")
            if target not in [ROBOT_ID, "all"]:
                logger.info("Commando niet voor deze robot (target: %s)", target)
                return
                
            # Haal bericht uit commando
            msg_content = command_data["data"].get("msg")
            
            # Verwerk EMERGENCY_STOP commando (hoogste prioriteit)
            if msg_content == "EMERGENCY_STOP":
                logger.warning("NOODSTOP GEACTIVEERD - robot stopt onmiddellijk")
                
                # Sla huidige doelpositie op voordat we stoppen
                LAST_TARGET_POS = TARGET_POS.copy()
                logger.info(f"Laatste doelpositie opgeslagen: ({LAST_TARGET_POS[0]}, {LAST_TARGET_POS[1]})")
                
                emergency_stop = True
                # Zet doelpositie op huidige positie om stil te staan
                pos = trans.getSFVec3f()
                TARGET_POS = [round(pos[0], 1), round(pos[1], 1)]
                # Leds uit
                turn_leds_off()
                return
                
            # Verwerk RESUME commando (om noodstop op te heffen)
            if msg_content == "RESUME":
                logger.info("NOODSTOP gedeactiveerd - robot kan weer bewegen")
                emergency_stop = False
                
                # Herstel de laatste doelpositie indien beschikbaar
                if LAST_TARGET_POS is not None:
                    logger.info(f"Beweging naar laatste doel hervatten: ({LAST_TARGET_POS[0]}, {LAST_TARGET_POS[1]})")
                    TARGET_POS = LAST_TARGET_POS
                    LAST_TARGET_POS = None
                    
                    # Leeg pad cache om herberekening van pad te forceren
                    path_cache = []
                return
                
            # Verwerk MOVE commando (alleen als er geen noodstop actief is)
            if not emergency_stop and isinstance(msg_content, dict) and msg_content.get("command") == "MOVE":
                target_pos = msg_content.get("target")
                if target_pos and "x" in target_pos and "y" in target_pos:
                    try:
                        # Valideer doelCoordinaten
                        x = float(target_pos["x"])
                        y = float(target_pos["y"])
                        x, y = validate_coordinates(x, y)
                        logger.info("MOVE commando ontvangen - nieuwe doelpositie: (%f, %f)", x, y)
                        TARGET_POS = [x, y]
                        # Leeg pad cache bij wijziging van doel
                        path_cache = []
                    except ValueError as ve:
                        logger.error("Ongeldige Coordinaten in MOVE commando: %s", ve)
    except json.JSONDecodeError as je:
        logger.error("Ongeldig JSON formaat in MQTT bericht: %s", je)
    except Exception as e:
        logger.error("Fout bij verwerken van MQTT commando: %s", e)

if mqtt_connected:
    # Abonneer op commando en status topics
    client.subscribe(TOPIC_COMMAND)
    client.message_callback_add(TOPIC_COMMAND, on_command)
    client.subscribe(TOPIC_STATUS)  # Abonneer op robot status berichten
    client.message_callback_add(TOPIC_STATUS, on_status)  # Voeg callback toe voor status berichten
    client.loop_start()
    logger.info(f"Geabonneerd op topics: {TOPIC_COMMAND}, {TOPIC_STATUS}")

# Bijhouden van laatste verzonden positie
last_sent_position = None

#  Detecteer obstakels met sensoren 
def detect_obstacles():
    try:
        obstacles = []
        # Lees sensorwaarden
        dN = sensor_N.getValue()
        dE = sensor_E.getValue()
        dS = sensor_S.getValue()
        dW = sensor_W.getValue()
        
        # Controleer op obstakels in elke richting
        if dN < OBSTACLE_THRESHOLD:
            obstacles.append("N")
        if dE < OBSTACLE_THRESHOLD:
            obstacles.append("E")
        if dS < OBSTACLE_THRESHOLD:
            obstacles.append("S")
        if dW < OBSTACLE_THRESHOLD:
            obstacles.append("W")
            
        logger.debug("Sensorwaarden -> N: %.2f, E: %.2f, S: %.2f, W: %.2f", dN, dE, dS, dW)
        return obstacles
    except Exception as e:
        logger.error("Fout bij obstakeldetectie: %s", e)
        return []

#  Schakel alle LEDs uit 
def turn_leds_off():
    # Schakel alle LED-indicators uit
    try:
        led_N.set(0)
        led_E.set(0)
        led_S.set(0)
        led_W.set(0)
        logger.debug("Alle LED's uitgeschakeld")
    except Exception as e:
        logger.error("Fout bij uitschakelen LED's: %s", e)

#  Stuur status via MQTT 
def send_status():
    global last_sent_position
    
    if not mqtt_connected:
        logger.warning("Kan status niet versturen: geen MQTT verbinding")
        return
        
    try:
        # Huidige positie ophalen
        pos = trans.getSFVec3f()  # [x, y, z]
        x_pos = round(pos[0], 1)
        y_pos = round(pos[1], 1)
        current_pos = (x_pos, y_pos)
        
        # Alleen versturen als positie is veranderd of elke 3 seconden (hartslag)
        current_time = time.time()
        if last_sent_position != current_pos or not hasattr(send_status, "last_heartbeat") or current_time - send_status.last_heartbeat >= 3.0:
            last_sent_position = current_pos
            send_status.last_heartbeat = current_time
            
            # Status bericht samenstellen
            status_message = {
                "protocolVersion": 1.0,
                "data": {
                    "sender": ROBOT_ID,
                    "target": "server",
                    "msg": {
                        "location": {"x": x_pos, "y": y_pos},
                        "obstacles": detect_obstacles(),
                        "emergency": emergency_stop
                    }
                }
            }
            
            # Naar JSON en versturen
            payload = json.dumps(status_message)
            client.publish(TOPIC_PUBLISH, payload)
            logger.info("Statusbericht verzonden: positie=(%f, %f), noodstop=%s",
                       x_pos, y_pos, emergency_stop)
    except Exception as e:
        logger.error("Fout bij verzenden status: %s", e)

#  Stel positie in 
def set_position(x, y):
    try:
        # Valideer en rond af
        new_x = round(x, 1)
        new_y = round(y, 1)
        
        # Houd binnen grenzen
        new_x = min(max(new_x, MIN_BOUND), MAX_BOUND)
        new_y = min(max(new_y, MIN_BOUND), MAX_BOUND)
        
        # Behoud huidige z-coördinaat
        current = trans.getSFVec3f()
        trans.setSFVec3f([new_x, new_y, current[2]])
        
        # Behoud rotatie (kijkend naar boven)
        rot.setSFRotation([0, 0, 1, 0])
        
        logger.debug("Positie ingesteld: (%f, %f)", new_x, new_y)
        return True
    except Exception as e:
        logger.error("Fout bij instellen positie: %s", e)
        return False

#  Manhattan distance heuristic 
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

#  Zoek dichtsbijzijnde valide positie 
def find_closest_valid_position(grid, pos):
    # Vind de dichtstbijzijnde geldige positie in het grid
    x, y = pos
    
    # Als de positie al geldig is, retourneer deze
    if 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT and grid[y][x] == 1:
        return pos
        
    # Zoek in uitbreidende vierkanten rond de positie
    for radius in range(1, max(GRID_WIDTH, GRID_HEIGHT)):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                # Alleen posities op de rand van het vierkant controleren
                if abs(dx) == radius or abs(dy) == radius:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and grid[ny][nx] == 1:
                        logger.info(f"Positie aangepast van ({x}, {y}) naar ({nx}, {ny})")
                        return (nx, ny)
                        
    # Als geen geldige positie gevonden, gebruik centrum van het grid als fallback
    logger.warning(f"Geen geldige positie gevonden bij ({x}, {y}), centrum van grid gebruikt")
    center_x, center_y = GRID_WIDTH // 2, GRID_HEIGHT // 2
    return (center_x, center_y)

#  World-grid coördinaatconversies 
def world_to_grid(x, y):
    # Zet Webots-positie om naar gridpositie
    gx = int(round(x / STEP_SIZE))
    gy = GRID_HEIGHT - 1 - int(round(y / STEP_SIZE))  # omgekeerde Y-as
    return gx, gy

def grid_to_world(gx, gy):
    # Zet gridpositie om naar Webots-positie
    x = round(gx * STEP_SIZE, 1)
    y = round((GRID_HEIGHT - 1 - gy) * STEP_SIZE, 1)
    return x, y

#  Markeer robot-obstakels op grid 
def mark_robot_obstacles(grid, other_robot_positions):
    # Markeer gridcellen die bezet zijn door andere robots en voeg veiligheidsmarges toe
    # Maak een tijdelijk grid
    temp_grid = [row[:] for row in grid]
    
    # Markeer cellen met andere robots als obstakels
    if other_robot_positions:
        for robot_id, pos_data in other_robot_positions.items():
            # Zet wereldcoordinaten om naar grid
            rx, ry = world_to_grid(pos_data["x"], pos_data["y"])
            
            # Controleer of coordinaten binnen grid vallen
            if 0 <= rx < GRID_WIDTH and 0 <= ry < GRID_HEIGHT:
                # Voeg een veiligheidsmarge toe rond robots
                for dx in range(-ROBOT_SAFETY_MARGIN, ROBOT_SAFETY_MARGIN + 1):
                    for dy in range(-ROBOT_SAFETY_MARGIN, ROBOT_SAFETY_MARGIN + 1):
                        # Alleen blokkeren als binnen veiligheidsafstand (Manhattan distance)
                        if abs(dx) + abs(dy) <= ROBOT_SAFETY_MARGIN:
                            nx, ny = rx + dx, ry + dy
                            # Alleen blokkeren als binnen grid grenzen
                            if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                                temp_grid[ny][nx] = 0  # Markeer als obstakel
                                logger.debug(f"Cel ({nx},{ny}) gemarkeerd als bezet door/nabij robot {robot_id}")
    
    return temp_grid

#  Voorspel toekomstige robotposities 
def predict_robot_positions(robot_positions, prediction_steps=PREDICTION_STEPS):
    # Als we nog geen historische gegevens hebben, retourneer alleen huidige posities
    if not hasattr(predict_robot_positions, "history"):
        predict_robot_positions.history = {}
    
    # Update geschiedenis
    for robot_id, pos_data in robot_positions.items():
        if robot_id not in predict_robot_positions.history:
            predict_robot_positions.history[robot_id] = []
        
        # Voeg huidige positie toe aan geschiedenis, houd laatste 3 posities
        predict_robot_positions.history[robot_id].append((pos_data["x"], pos_data["y"]))
        if len(predict_robot_positions.history[robot_id]) > 3:
            predict_robot_positions.history[robot_id].pop(0)
    
    # Verwijder verouderde robots (niet gezien in huidige gegevens)
    for robot_id in list(predict_robot_positions.history.keys()):
        if robot_id not in robot_positions:
            del predict_robot_positions.history[robot_id]
    
    # Voorspel toekomstige posities
    predicted_positions = {}
    for robot_id, history in predict_robot_positions.history.items():
        if len(history) >= 2:
            # Bereken bewegingsvector op basis van laatste twee posities
            current_x, current_y = history[-1]
            prev_x, prev_y = history[-2]
            dx = current_x - prev_x
            dy = current_y - prev_y
            
            # Alleen voorspellen als robot daadwerkelijk beweegt
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                # Voorspel toekomstige posities
                for step in range(1, prediction_steps + 1):
                    future_x = current_x + dx * step
                    future_y = current_y + dy * step
                    
                    # Zorg voor geldige coordinaten
                    future_x = min(max(future_x, 0.0), 0.9)
                    future_y = min(max(future_y, 0.0), 0.9)
                    
                    # Voeg voorspelde positie toe
                    key = f"{robot_id}_pred_{step}"
                    predicted_positions[key] = {
                        "x": future_x,
                        "y": future_y,
                        "timestamp": robot_positions[robot_id]["timestamp"]
                    }
    
    # Combineer huidige en voorspelde posities
    combined = robot_positions.copy()
    combined.update(predicted_positions)
    return combined

#  Dijkstra padzoekalgoritme met robotvermijding 
def dijkstra(grid, start, goal, other_robot_positions=None):
    # Haal grid met gemarkeerde robotobstakels indien nodig
    if other_robot_positions:
        temp_grid = mark_robot_obstacles(grid, other_robot_positions)
    else:
        temp_grid = [row[:] for row in grid]  # Maak een diepe kopie
    
    # Controleer of start of doel nu een obstakel is (vanwege robotmarkering)
    if (0 <= start[0] < GRID_WIDTH and 0 <= start[1] < GRID_HEIGHT and 
        0 <= goal[0] < GRID_WIDTH and 0 <= goal[1] < GRID_HEIGHT):
        # Zorg ervoor dat start en doel niet als obstakels worden gemarkeerd
        temp_grid[start[1]][start[0]] = 1  # y,x volgorde
        temp_grid[goal[1]][goal[0]] = 1

    
    # Setup voor A* algoritme (Dijkstra is A* met h=0)
    queue = []
    heapq.heappush(queue, (0, start))
    visited = set()
    came_from = {}
    cost_so_far = {start: 0}
    
    while queue:
        current_cost, current = heapq.heappop(queue)
        
        if current in visited:
            continue
            
        visited.add(current)
        
        if current == goal:
            break
            
        x, y = current
        
        # Controleer alle vier richtingen
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        
        for nx, ny in neighbors:
            if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                if temp_grid[ny][nx] == 1 and (nx, ny) not in visited:
                    new_cost = current_cost + 1
                    if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                        cost_so_far[(nx, ny)] = new_cost
                        priority = new_cost + heuristic((nx, ny), goal)
                        heapq.heappush(queue, (priority, (nx, ny)))
                        came_from[(nx, ny)] = current
    
    # Padreconstructie
    if goal not in came_from and goal != start:
        # Geen direct pad gevonden, probeer gedeeltelijk pad te vinden
        logger.warning(f"Geen direct pad gevonden naar ({goal[0]}, {goal[1]}), zoeken naar dichtstbijzijnde bereikbare punt")
        if not visited:
            logger.error("Geen bereikbare punten gevonden")
            return []
            
        # Vind de dichtstbijzijnde bezochte knoop bij het doel
        closest_node = min(visited, key=lambda node: heuristic(node, goal))
        
        if closest_node == start:
            logger.error(f"Kan geen geldig pad vinden richting ({goal[0]}, {goal[1]})")
            return []
            
        # Reconstrueer pad naar de dichtstbijzijnde bereikbare knoop
        path = []
        node = closest_node
        while node != start:
            path.append(node)
            node = came_from.get(node)
            if node is None:
                return []  # Dit zou niet moeten gebeuren, maar voor de zekerheid
        
        path.reverse()
        return path # Werkt voor nu
    
    # Normale padreconstructie als pad bestaat
    path = []
    node = goal
    
    # Als we al bij het doel zijn
    if start == goal:
        return []
        
    while node != start:
        path.append(node)
        node = came_from.get(node)
        if node is None:
            logger.error(f"Padreconstructie mislukt van {start} naar {goal}")
            return []
            
    path.reverse()
    return path

#  Pad cache om huidig pad op te slaan 
path_cache = []

#  Beweeg naar doel met botsingsvermijding 
def move_to_target():
    global path_cache
    
    if emergency_stop:
        logger.info("NOODSTOP actief - geen beweging toegestaan")
        return
        
    pos = trans.getSFVec3f()
    current_gx, current_gy = world_to_grid(pos[0], pos[1])
    target_gx, target_gy = world_to_grid(TARGET_POS[0], TARGET_POS[1])
    
    # Verwijder oude robotposities (ouder dan 5 seconden)
    current_time = time.time()
    for robot_id in list(other_robots.keys()):
        if current_time - other_robots[robot_id].get("timestamp", 0) > 5:
            logger.info(f"Verouderde positiegegevens voor {robot_id} verwijderd")
            del other_robots[robot_id]
    
    # Controleer of we het doel hebben bereikt
    if (current_gx, current_gy) == (target_gx, target_gy):
        logger.info(f"Doel bereikt: ({target_gx}, {target_gy})")
        return
    
    # Bepaal of we het pad opnieuw moeten berekenen
    recalculate = False
    
    # Als pad leeg is of doel is veranderd: herbereken
    if not path_cache or (target_gx, target_gy) != path_cache[-1] if path_cache else True:
        recalculate = True
        logger.info("Pad leeg of doel veranderd, herberekening nodig")
    
    # Voorspel toekomstige posities van andere robots
    predicted_robots = predict_robot_positions(other_robots) if other_robots else {}
    
    # Controleer of er een robot in ons pad is of wordt voorspeld
    if path_cache and not recalculate:
        for robot_id, pos_data in predicted_robots.items():
            # Controleer of een robot in ons pad is
            rx, ry = world_to_grid(pos_data["x"], pos_data["y"])
            for path_node in path_cache:
                # Controleer niet alleen exacte locatie maar ook nabijheid
                if abs(rx - path_node[0]) <= 1 and abs(ry - path_node[1]) <= 1:
                    logger.info(f"Robot {robot_id} gedetecteerd in pad (of voorspeld). Herberekenen...")
                    recalculate = True
                    break
            if recalculate:
                break
    
    # Controleer of we moeten wachten voor robots met hogere prioriteit
    should_wait = False
    for robot_id, pos_data in other_robots.items():
        if should_yield_to_robot(ROBOT_ID, robot_id):
            # Bereken afstand tussen robots
            my_pos = (pos[0], pos[1])
            other_pos = (pos_data["x"], pos_data["y"])
            distance = ((my_pos[0] - other_pos[0])**2 + (my_pos[1] - other_pos[1])**2)**0.5
            
            # Als robots dicht bij elkaar zijn, wacht de robot met lagere prioriteit
            if distance < ROBOT_PROXIMITY_THRESHOLD:  # Pas drempel aan indien nodig
                logger.info(f"Voorrang geven aan robot met hogere prioriteit: {robot_id}")
                should_wait = True
                break
    
    if should_wait:
        # Wacht gewoon op je plaats
        logger.info("Wachten tot robot met hogere prioriteit passeert")
        return
    
    # Herbereken pad indien nodig
    if recalculate:
        logger.info(f"Pad berekenen van ({current_gx},{current_gy}) naar ({target_gx},{target_gy})")
        
        # Probeer eerst met voorspelde robotposities
        if predicted_robots:
            logger.info("Pad berekenen met voorspelde robotposities")
            path_cache = dijkstra(GRID, (current_gx, current_gy), (target_gx, target_gy), predicted_robots)
        
        # Als dat mislukt, probeer alleen met huidige posities
        if not path_cache and other_robots:
            logger.warning("Geen pad gevonden met voorspellingen, proberen met alleen huidige posities")
            path_cache = dijkstra(GRID, (current_gx, current_gy), (target_gx, target_gy), other_robots)
        
        # Als laatste redmiddel, probeer zonder robotvermijding
        if not path_cache:
            logger.warning("Geen pad gevonden met robotvermijding, proberen zonder vermijding")
            path_cache = dijkstra(GRID, (current_gx, current_gy), (target_gx, target_gy))
            
        if not path_cache:
            logger.error(f"Geen pad kon worden gevonden naar ({target_gx}, {target_gy})")
            # Noodoplossing: probeer een kleine stap in de richting van het doel
            dx = 1 if target_gx > current_gx else -1 if target_gx < current_gx else 0
            dy = 1 if target_gy > current_gy else -1 if target_gy < current_gy else 0
            
            # Probeer verschillende richtingen als noodoplossing
            for direction in [(dx, dy), (dx, 0), (0, dy), (1, 0), (0, 1), (-1, 0), (0, -1)]:
                nx, ny = current_gx + direction[0], current_gy + direction[1]
                # Controleer of deze richting geldig is
                if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and GRID[ny][nx] == 1:
                    path_cache = [(nx, ny)]
                    logger.info(f"Noodpad gevonden: één stap in richting ({direction[0]},{direction[1]})")
                    break
            else:
                logger.error("Robot zit volledig vast, geen geldige bewegingen mogelijk")
                return
            
        logger.info(f"Pad berekend met {len(path_cache)} stappen")
    
    # Als we een pad hebben om te volgen
    if path_cache:
        next_step = path_cache.pop(0)
        new_x, new_y = grid_to_world(*next_step)
        
        # Verplaats robot
        if set_position(new_x, new_y):
            dx = next_step[0] - current_gx
            dy = next_step[1] - current_gy
            dir_map = {(0,1): "N", (1,0): "E", (0,-1): "S", (-1,0): "W"}
            direction = dir_map.get((dx, dy), "?")
            
            # LED-toewijzingen
            led_N.set(1 if direction == "N" else 0)
            led_E.set(1 if direction == "E" else 0)
            led_S.set(1 if direction == "S" else 0)
            led_W.set(1 if direction == "W" else 0)
            
            logger.info("Beweging naar grid (%d,%d) wereld (%.1f, %.1f) richting %s",
                        next_step[0], next_step[1], new_x, new_y, direction)
        else:
            logger.error(f"Kan positie niet instellen op ({new_x}, {new_y})")

#  Hoofdlus 
logger.info("Simulatie gestart")
last_movement_time = time.time()

try:
    while robot.step(timestep) != -1:
        current_time = time.time()
        
        # Elke seconde bewegen en status versturen
        if current_time - last_movement_time >= 1.0:
            move_to_target()
            send_status()
            last_movement_time = current_time
            
        # Korte pauze voor CPU belasting
        time.sleep(0.1)
except KeyboardInterrupt:
    logger.info("Simulatie handmatig gestopt")
except Exception as e:
    logger.critical("Onverwachte fout: %s", e)
finally:
    # Opruimen bij afsluiten
    if mqtt_connected:
        client.loop_stop()
        client.disconnect()
        logger.info("MQTT verbinding afgesloten")
    logger.info("Simulatie beëindigd")
