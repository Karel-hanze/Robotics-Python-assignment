# Implements communication with Webots via Serial over USB.
# Met geïntegreerd Dijkstra's algoritme voor kortste-pad-berekening.
# Werkt met de "line_following_with_HIL" controller in Webots.

# Getest met MicroPython v1.25.0 op ESP32 WROOM 32 dev kit board
# Webots R2023a, Windows 11 met Python 3.10.5 64‐bit

from machine import Pin, UART
from time import sleep

########### 1) GPIO & UART‐setup zoals voorheen ###########

led_board  = Pin(2, Pin.OUT)     # Onboard LED ESP32
led_yellow = Pin(4, Pin.OUT)
led_blue   = Pin(23, Pin.OUT)
led_green  = Pin(22, Pin.OUT)
led_red    = Pin(21, Pin.OUT)
button_left  = Pin(34, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)

# Wacht op linker knop vóór switch naar UART1
print("Klik op de linkerknoop op de ESP32 om te beginnen.")
print("Sluit daarna Thonny af en start de Webots‐simulatie.")
print("Of klik STOP in Thonny om terug te gaan naar de REPL.")
while not button_left():
    sleep(0.25)
    led_board.value(not led_board())

# Zet UART1 op (zelfde pins als UART0) om via USB met Webots te communiceren
uart = UART(1, 115200, tx=1, rx=3)

########### 2) DEFINITIE VAN DE GRAPH & DIJKSTRA ###########

GRAPH = {
    'B1': {'K1': 0.1},
    'B2': {'K2': 0.1},
    'B3': {'K3': 0.1},
    'B4': {'K4': 0.1},
    'M1': {'K1': 0.195, 'M2': 0.5, 'M4': 0.15},
    'M2': {'M1': 0.5, 'M3': 0.15},
    'M3': {'M2': 0.15, 'M4': 0.5, 'M7': 0.1},
    'M4': {'M1': 0.15, 'M3': 0.5, 'M6': 0.1},
    'M5': {'K4': 0.25, 'M6': 0.5, 'M9': 0.1},
    'M6': {'M4': 0.1, 'M5': 0.5, 'M7': 0.5, 'M8': 0.1},
    'M7': {'K5': 0.25, 'M3': 0.1, 'M6': 0.5},
    'M8': {'M6': 0.1, 'M9': 0.5, 'M11': 0.15},
    'M9': {'M5': 0.1, 'M8': 0.5, 'M10': 0.15},
    'M10': {'M9': 0.15, 'M11': 0.5},
    'M11': {'K8': 0.195, 'M8': 0.15, 'M10': 0.5},
    'K1': {'K2': 0.1, 'M1': 0.195, 'B1': 0.1},
    'K2': {'K1': 0.1, 'K3': 0.1, 'B2': 0.1},
    'K3': {'K2': 0.1, 'K4': 0.1, 'B3': 0.1},
    'K4': {'K3': 0.1, 'M5': 0.25, 'B4': 0.1},
    'K5': {'K6': 0.1, 'M7': 0.25, 'E1': 0.1},
    'K6': {'K5': 0.1, 'K7': 0.1, 'E2': 0.1},
    'K7': {'K6': 0.1, 'K8': 0.1, 'E3': 0.1},
    'K8': {'K7': 0.1, 'M11': 0.195, 'E4': 0.1},
    'E1': {'K5': 0.1},
    'E2': {'K6': 0.1},
    'E3': {'K7': 0.1},
    'E4': {'K8': 0.1}
}

def dijkstra(graph, start, goal):
    """
    Bereken kortste pad van start naar goal in graph.
    graph is een dict van dicts: graph[node][neighbor] = gewicht.
    Retourneert: (afstand, lijst_van_knooppunten_in_pad)
    """
    # Init-structuren
    unvisited = set(graph.keys())
    # afstand[n] = voorlopig kortsteberekende afstand van start tot n
    afstand = {node: float('inf') for node in graph}
    afstand[start] = 0
    # parent[n] = voorgaand knooppunt op weg naar n
    parent = {node: None for node in graph}

    while unvisited:
        # Kies onbezochte met kleinste afstand
        huidige = min(unvisited, key=lambda node: afstand[node])
        if afstand[huidige] == float('inf'):
            break  # onbereikbare knooppunten
        unvisited.remove(huidige)

        if huidige == goal:
            break  # we hebben de bestemming bereikt

        # Relax alle buren
        for buur, gewicht in graph[huidige].items():
            new_dist = afstand[huidige] + gewicht
            if new_dist < afstand[buur]:
                afstand[buur] = new_dist
                parent[buur] = huidige

    # Bouw het pad op door parent‐pointer terug te volgen
    pad = []
    node = goal
    if parent[node] is not None or node == start:
        while node is not None:
            pad.insert(0, node)
            node = parent[node]
    return afstand[goal], pad

########### 3) UITVOER KORTSTE PAD VOORAF ###########

# Stel begin‐ en eindknooppunt in
SOURCE_NODE = 'B1'
DEST_NODE   = 'E1'

# Reken het kortste pad uit
afst, pad = dijkstra(GRAPH, SOURCE_NODE, DEST_NODE)

if pad:
    print("Kortste pad van", SOURCE_NODE, "naar", DEST_NODE, "is:")
    print("->".join(pad), "(afstand:", afst, ")")
    # Voorbeeld: stuur het pad als comma‐gescheiden string naar Webots
    uart.write("PATH:" + ",".join(pad) + "\n")
    
else:
    print("Geen pad gevonden van", SOURCE_NODE, "naar", DEST_NODE)
    

########### 4) Bestaande LINE‐FOLLOWING STATE‐MACHINE ###########

# Sensorstatus (wordt geüpdatet door Webots via serial)
line_left   = False
line_center = False
line_right  = False

# State‐machine variabelen
current_state = 'forward'
counter       = 0
COUNTER_MAX   = 5
COUNTER_STOP  = 50
state_updated = True

while True:
    ##################   See: Lees sensors via serial ###################
    if uart.any():
        msg_bytes = uart.read()              # Lees alle binnengekomen bytes
        msg_str   = msg_bytes.decode('utf-8') # Zet om naar string

        # We nemen aan dat Webots achteraan een string stuurt met 3 cijfers:
        #   ...XYZ\n  waarbij X=waarde line_left, Y=waarde line_center, Z=waarde line_right
        # Controleer de laatste karakters (voorafgegaan door eventuele "\r\n")
        msg_str = msg_str.strip()  # knip newline/whitespace weg
        if msg_str == "Request Path":
            path_str = "PATH:" + ",".join(pad) + "\n"
            uart.write(path_str.encode('utf-8'))

        # Bescherm tegen lege of onverwachte berichten
        if len(msg_str) >= 3:
            # Interpretatie: laatste 3 chars zijn '101', '010', etc.
            # line_left  = True als voorlaatste vierde char == '1'
            if msg_str[-3] == '1':
                line_left = True
                led_blue.on()
            else:
                line_left = False
                led_blue.off()
            if msg_str[-2] == '1':
                line_center = True
                led_green.on()
            else:
                line_center = False
                led_green.off()
            if msg_str[-1] == '1':
                line_right = True
                led_red.on()
            else:
                line_right = False
                led_red.off()

    ##################   Think: State‐machine ###################
    # Voor elke transitie zetten we state_updated = True om nieuwe staat te verzenden
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center:  # lijn kwijt
            current_state = 'turn_left'
            state_updated = True
        elif button_right.value():
            current_state = 'stop'
            state_updated = True

    elif current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value():
            current_state = 'stop'
            state_updated = True

    elif current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value():
            current_state = 'stop'
            state_updated = True

    elif current_state == 'stop':
        led_board.value(1)  # onboard LED branden
        if counter >= COUNTER_STOP:
            current_state  = 'forward'
            state_updated  = True
            led_board.value(0)

    ##################   Act: verzend nieuwe staat ###################
    if state_updated:
        uart.write(current_state + "\n")
        state_updated = False

    counter += 1
    sleep(0.02)
