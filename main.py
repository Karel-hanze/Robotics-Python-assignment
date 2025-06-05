# main.py for ESP32 MicroPython

# Implements communication with Webots via Serial over USB.
# The ESP32 now acts solely as a sensor reporter.
# All motor control is handled directly by Webots for the simulated robot.

# Tested with MicroPython v1.25.0 on ESP32 WROOM 32 dev kit board
# communicating with Webots R2023a, on Windows 11 running Python 3.10.5 64-bit

# Author: Felipe N. Martins
# Date: 05 June 2025
# Last update: 05 June 2025 (ESP32 is sensor-only, no motor control, no state sending)


###### IMPORTANT: Close Thonny after starting this code! #######
# This is necessary because the serial port cannot be used by
# two different programs (Thonny and Webots) at the same time.


from machine import Pin
from time import sleep
from sys import stdin, stdout


# --- Hardware Definitions ---
# Define ESP32 onboard LED (often GPIO2, check your board)
led_board = Pin(2, Pin.OUT)
# Define external LEDs (if connected), adjust pins as needed
# These can still be used to visualize sensor states on the physical ESP32
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)

# Define buttons (check your board's pins for onboard buttons, or your wiring for external)
button_left = Pin(34, Pin.IN, Pin.PULL_DOWN) # Use this as the "START" button
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN) # Use this as the "STOP" button

# --- NO MOTOR CONTROL SETUP ---
# All motor related pin definitions, PWM setup, and set_motor_speed function are REMOVED.


# --- Initialization Sequence (Waiting for Button Press) ---
print("ESP32: Ready. Press the START button (left button) to signal Webots.")
print("Once pressed, close Thonny and run the Webots simulation.")
print("You can also press the STOP button (right button) at any time to exit.")

while button_left.value() == False:
    sleep(0.25)
    led_board.value(not led_board.value())

print("ESP32: START button pressed! Signaling Webots and entering main loop.")
sleep(0.1)

# --- Handshake: Signal Webots that ESP32 is ready ---
stdout.write(b'ESP32_READY\n')
print("ESP32: 'ESP32_READY' signal sent to Webots.")
sleep(0.5) # Give Webots a moment to receive this

# --- Global buffer for incoming serial data (though not expecting motor commands now) ---
received_line_buffer = ""

# --- Main Loop: Continuously Read, Send Sensor Data ---
while True:

    # Check for STOP button press at any time
    if button_right.value() == True:
        print("ESP32: STOP button pressed! Exiting main loop and returning to REPL.")
        sleep(0.1)
        led_board.value(0) # Turn off all LEDs on exit
        led_yellow.value(0)
        led_blue.value(0)
        led_green.value(0)
        led_red.value(0)
        break

    # --- Read Sensor Data (Assuming actual line sensors are connected to these pins) ---
    # These GPIOs would be connected to the output of your line sensors.
    # Adjust pin numbers based on your actual wiring.
    # For a direct translation of Webots' `gsValues > 600` logic:
    # Let's assume typical reflectance sensors where HIGH means 'dark' (on line).
    # You might need to invert this logic based on your sensor's output (active high/low).
    # For now, mimicking the '1' for line detected.
    
    # Placeholder for actual sensor readings. You'll need to define how your
    # physical line sensors are read (e.g., analog to digital conversion, or digital input).
    # If using digital line sensors:
    # line_left_pin = Pin(36, Pin.IN) # Example digital pin for left sensor
    # line_center_pin = Pin(39, Pin.IN) # Example digital pin for center sensor
    # line_right_pin = Pin(34, Pin.IN) # Example digital pin for right sensor (re-using for example)

    # For the purpose of this example, assuming line_left, line_center, line_right
    # would be determined by *your* physical sensor reading code here.
    # Since we removed motor control, we don't *need* to update these on ESP32,
    # but we will send them to Webots. For a pure sensor reporter, just send what you read.

    # Simulating sensor readings for the example (you'd replace this with real reads)
    # If you remove this, ensure you assign proper values to line_left, line_center, line_right
    # from your actual hardware sensor inputs.
    # For now, let's make them dependent on the time, or a button for testing:
    # Example: If your line sensors are connected to GPIOs 36, 39, 34 respectively.
    # line_left = Pin(36, Pin.IN).value()
    # line_center = Pin(39, Pin.IN).value()
    # line_right = Pin(34, Pin.IN).value()

    # For demonstration without actual sensors, using buttons to simulate:
    line_left = button_left.value() # If left button pressed, simulate line on left
    line_center = led_board.value() # If board LED is on, simulate line in center (heartbeat)
    line_right = button_right.value() # If right button pressed, simulate line on right

    # Update physical LEDs on ESP32 based on "sensor" readings (for debugging)
    led_blue.value(line_left)
    led_green.value(line_center)
    led_red.value(line_right)
    
    # Prepare the sensor message (e.g., "010" for line center)
    sensor_message_str = ''
    sensor_message_str += '1' if line_left else '0'
    sensor_message_str += '1' if line_center else '0'
    sensor_message_str += '1' if line_right else '0'
    
    # Send sensor data to Webots
    stdout.write(f'{sensor_message_str}\n'.encode('UTF-8'))
    # print(f"ESP32: Sent sensor data: {sensor_message_str}") # For debugging in Thonny

    # --- Read incoming characters from stdin (UART0) ---
    # We still read, just in case Webots sends anything, but we don't expect motor commands.
    while True:
        ch = stdin.read(1)
        if ch is None:
            break
        received_line_buffer += ch
        if ch == '\n':
            received_data = received_line_buffer.strip()
            received_line_buffer = ""
            # print(f"ESP32: Received unexpected data: '{received_data}'") # Debug any unexpected messages
            break # Process this line and then check for more

    # A short delay for communication cycle
    sleep(0.02)