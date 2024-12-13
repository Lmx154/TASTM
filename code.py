import time
import usb_cdc
import random
import math
import board
import digitalio

# USB CDC setup
serial = usb_cdc.data

# Button setup
button = digitalio.DigitalInOut(board.D5)  # Replace D5 with your GPIO pin
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP  # Use pull-up resistor

def time_date_sync():
    """Generates the current time and date in the format: YYYY|MM|DD|Weekday|HH:MM:SS"""
    now = time.localtime()
    year = now.tm_year
    month = now.tm_mon
    day = now.tm_mday
    weekday = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"][now.tm_wday]
    hours = now.tm_hour
    minutes = now.tm_min
    seconds = now.tm_sec
    return f"{year}|{month:02}|{day:02}|{weekday}|{hours:02}:{minutes:02}:{seconds:02}"

def update_speed(thrust, drag, gravity, mass, delta_time, current_speed):
    """Calculates the updated speed of the rocket."""
    acceleration = (thrust - drag - gravity) / mass
    return current_speed + acceleration * delta_time

def update_altitude(current_altitude, speed, delta_time):
    """Calculates the updated altitude based on current speed and time interval."""
    return current_altitude + speed * delta_time

def update_environmental_factors(altitude):
    """Adjusts environmental factors like pressure and temperature based on altitude."""
    sea_level_pressure = 1013.25  # hPa
    scale_height = 8500  # meters
    pressure = sea_level_pressure * math.exp(-altitude / scale_height)

    sea_level_temperature = 288.15  # Kelvin
    lapse_rate = -0.0065  # K/m
    temperature = sea_level_temperature + lapse_rate * altitude
    temperature = max(temperature, 216.65)  # Minimum ~ -57°C in the stratosphere

    humidity = max(0, 50 - (altitude / 200))  # Example linear decrease
    return pressure, temperature, humidity

class RocketSimulation:
    def __init__(self):
        self.reset()

    def reset(self):
        """Resets the rocket to its initial standby state."""
        self.altitude = 0.0
        self.speed = 0.0
        self.mass = 20.0
        self.thrust = 500.0
        self.gravity = 9.81
        self.drag_coefficient = 0.05
        self.delta_time = 0.25
        self.state = "standby"

    def simulate_step(self):
        """Simulates one step of the rocket launch."""
        drag = self.drag_coefficient * self.speed ** 2
        self.speed = update_speed(self.thrust, drag, self.gravity * self.mass, self.mass, self.delta_time, self.speed)
        self.altitude = update_altitude(self.altitude, self.speed, self.delta_time)
        pressure, temperature, humidity = update_environmental_factors(self.altitude)

        accel_x = round(self.thrust / self.mass - self.gravity, 2)
        accel_y = round(random.uniform(-0.5, 0.5), 2)
        accel_z = round(random.uniform(-0.5, 0.5), 2)
        gx = round(random.uniform(80.0, 120.0), 2)
        gy = round(random.uniform(-70.0, -50.0), 2)
        gz = round(random.uniform(60.0, 80.0), 2)
        latitude = 32.9394
        longitude = -106.922112
        satellites = 8

        return f"{time_date_sync()}|{accel_x}|{accel_y}|{accel_z}|{gx}|{gy}|{gz}|{round(temperature - 273.15, 2)}|{round(temperature * 9/5 - 459.67, 2)}|{round(pressure, 2)}|{round(self.altitude, 2)}|{round(humidity, 2)}|1.0|2|{latitude}|{longitude}|{round(self.speed, 2)}|{round(self.altitude, 2)}|{satellites}"

    def standby_mode(self):
        """Simulates the rocket on standby."""
        latitude = 32.9394
        longitude = -106.922112
        altitude = 0.0
        pressure, temperature, humidity = update_environmental_factors(altitude)

        accel_x = 0.0
        accel_y = 0.0
        accel_z = 0.0
        gx = 0.0
        gy = 0.0
        gz = 0.0
        satellites = 8

        return f"{time_date_sync()}|{accel_x}|{accel_y}|{accel_z}|{gx}|{gy}|{gz}|{round(temperature - 273.15, 2)}|{round(temperature * 9/5 - 459.67, 2)}|{round(pressure, 2)}|{round(altitude, 2)}|{round(humidity, 2)}|1.0|2|{latitude}|{longitude}|0.0|{round(altitude, 2)}|{satellites}"

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
                else:
                    rocket_sim.reset()

        if rocket_sim.state == "standby":
            data = rocket_sim.standby_mode()
        else:
            data = rocket_sim.simulate_step()

        serial.write((data + "\n").encode("utf-8"))
        if usb_cdc.console:
            usb_cdc.console.write(f"Sent: {data}\n".encode("utf-8"))
        time.sleep(0.25)
except KeyboardInterrupt:
    print("Simulation stopped.")
