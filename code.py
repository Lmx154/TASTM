import time
import usb_cdc
import random
import math
import board
import digitalio
import neopixel  # Import the NeoPixel library

# USB CDC setup
serial = usb_cdc.data

# Button setup
button = digitalio.DigitalInOut(board.D5)  # Replace D5 with your GPIO pin
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP  # Use pull-up resistor

# NeoPixel setup
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)  # Initialize NeoPixel with 1 LED
pixel.brightness = 0.3  # Set brightness (0.0 to 1.0)

def time_date_sync():
    """Generates the current time and date in the format: YYYY/MM/DD (Weekday) HH:MM:SS"""
    now = time.localtime()
    year = now.tm_year
    month = now.tm_mon
    day = now.tm_mday
    weekday = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"][now.tm_wday]
    hours = now.tm_hour
    minutes = now.tm_min
    seconds = now.tm_sec
    return f"{year}/{month:02}/{day:02} ({weekday}) {hours:02}:{minutes:02}:{seconds:02}"

def format_message(data, rssi, snr):
    """Formats the telemetry message based on the specified key output format."""
    message = (
        "[{timestamp}] {accel_x},{accel_y},{accel_z},"
        "{gx},{gy},{gz},{temp_c},{temp_f},{pressure},"
        "{altitude},{humidity},1,2,{latitude},{longitude},"
        "{speed},{altitude},8"
    ).format(**data)
    
    length = len(message)
    return (
        "$Message length: {}\r\n"
        "Message: {}\r\n"
        "RSSI: {}\r\n"
        "Snr: {:.2f}\r\n"
    ).format(length, message, rssi, snr)


def simulate_data():
    """Generates simulated telemetry data."""
    return {
        "timestamp": time_date_sync(),
        "accel_x": round(random.uniform(-0.5, 0.5), 2),
        "accel_y": round(random.uniform(-0.5, 0.5), 2),
        "accel_z": round(random.uniform(-9.8, -9.7), 2),
        "gx": round(random.uniform(10, 20), 2),
        "gy": round(random.uniform(-10, -5), 2),
        "gz": round(random.uniform(60, 80), 2),
        "temp_c": round(25 + random.uniform(-5, 5), 2),
        "temp_f": round((25 + random.uniform(-5, 5)) * 1.8 + 32, 2),
        "pressure": round(1010 + random.uniform(-10, 10), 2),
        "altitude": round(random.uniform(-50, 50), 2),
        "humidity": random.randint(20, 70),
        "latitude": 26.273800,
        "longitude": -98.431976,
        "speed": round(random.uniform(0.1, 0.5), 2),
    }

class RocketSimulation:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the rocket to its initial standby state."""
        self.state = "standby"

    def standby_mode(self):
        """Simulates the rocket on standby."""
        return simulate_data()

    def simulate_step(self):
        """Simulates one step of the rocket launch."""
        return simulate_data()

if not serial:
    print("No serial connection available.")
    while True:
        pass

print("Starting rocket telemetry simulation...")

rocket_sim = RocketSimulation()
button_state = button.value

try:
    while True:
        current_button_state = button.value
        if current_button_state != button_state:
            button_state = current_button_state
            if not button_state:  # Button pressed
                if rocket_sim.state == "standby":
                    rocket_sim.state = "launch"
                    pixel.fill((0, 255, 0))  # Green for launch mode
                else:
                    rocket_sim.reset()
                    pixel.fill((255, 0, 0))  # Red for standby mode

        if rocket_sim.state == "standby":
            data = rocket_sim.standby_mode()
        else:
            data = rocket_sim.simulate_step()

        rssi = random.randint(-100, -90)
        snr = round(random.uniform(7, 10), 2)
        formatted_message = format_message(data, rssi, snr)

        serial.write(formatted_message.encode("utf-8"))
        if usb_cdc.console:
            usb_cdc.console.write(f"Sent: {formatted_message}\r\n".encode("utf-8"))
            usb_cdc.console.flush()
        time.sleep(0.25)
except KeyboardInterrupt:
    print("Simulation stopped.")

