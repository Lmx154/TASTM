import time
import usb_cdc
import random
import math
import board
import digitalio
import neopixel

# USB CDC setup
serial = usb_cdc.data

# Button setup
button = digitalio.DigitalInOut(board.D5)  # Replace D5 with your GPIO pin
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP  # Use pull-up resistor

# NeoPixel setup
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)  # Initialize NeoPixel with 1 LED
pixel.brightness = 0.2  # Set brightness (0.0 to 1.0)

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

class RocketSimulation:
    def __init__(self, scenario="sunny_texas"):
        self.scenario = scenario
        self.reset()

    def reset(self):
        """Resets the rocket to its initial standby state and applies scenario conditions."""
        self.altitude = 0.0
        self.speed = 0.0
        self.mass = 20.0
        self.thrust = 500.0
        self.gravity = 9.81
        self.base_temp = 288.15  # baseline temperature in Kelvin at sea level (15°C)
        self.base_humidity = 50.0
        self.drag_coefficient = 0.05
        self.delta_time = 0.25
        self.state = "standby"

        # Apply scenario conditions
        self.apply_scenario_conditions()

    def apply_scenario_conditions(self):
        """Adjust environment and drag based on scenario."""
        if self.scenario == "sunny_texas":
            # Hotter, lower humidity -> less dense air, slightly less drag
            # Let's say ~35°C: 35 + 273.15 = 308.15 K
            # Humidity ~20%
            # Slightly lower drag due to less dense air
            self.base_temp = 308.15
            self.base_humidity = 20.0
            self.drag_coefficient = 0.045
        elif self.scenario == "cold_maine":
            # Colder, higher humidity -> more dense air, slightly more drag
            # Let's say -10°C: -10 + 273.15 = 263.15 K
            # Humidity ~80%
            # Slightly higher drag due to denser air
            self.base_temp = 263.15
            self.base_humidity = 80.0
            self.drag_coefficient = 0.055
        else:
            # Default conditions if scenario not recognized
            self.base_temp = 288.15
            self.base_humidity = 50.0
            self.drag_coefficient = 0.05

    def simulate_step(self):
        """Simulates one step of the rocket launch."""
        drag = self.drag_coefficient * self.speed ** 2
        self.speed = self.update_speed(drag)
        self.altitude = self.update_altitude()
        pressure, temperature, humidity = self.update_environmental_factors()
        data = self.generate_data(pressure, temperature, humidity)
        return data

    def standby_mode(self):
        """Simulates the rocket on standby."""
        pressure, temperature, humidity = self.update_environmental_factors(altitude=0)
        data = self.generate_data(pressure, temperature, humidity, standby=True)
        return data

    def update_speed(self, drag):
        """Calculates the updated speed of the rocket."""
        acceleration = (self.thrust - drag - self.gravity * self.mass) / self.mass
        return self.speed + acceleration * self.delta_time

    def update_altitude(self):
        """Calculates the updated altitude based on current speed and time interval."""
        return self.altitude + self.speed * self.delta_time

    def update_environmental_factors(self, altitude=None):
        """Adjusts environmental factors like pressure, temperature, humidity based on altitude and scenario."""
        if altitude is None:
            altitude = self.altitude
        sea_level_pressure = 1013.25  # hPa
        scale_height = 8500  # meters

        # Pressure decreases with altitude
        pressure = sea_level_pressure * math.exp(-altitude / scale_height)

        # Temperature: start from scenario base and apply lapse rate
        # If scenario is Texas: it's already hotter at sea level
        # If scenario is Maine: it's already colder
        lapse_rate = -0.0065  # K/m
        temperature = self.base_temp + lapse_rate * altitude
        temperature = max(temperature, 216.65)

        # Humidity: start from scenario base, adjust with altitude (less humidity as you go higher)
        # We'll just linearly decrease humidity as before
        humidity = max(0, self.base_humidity - (altitude / 200))

        return pressure, temperature, humidity

    def generate_data(self, pressure, temperature, humidity, standby=False):
        """Generates telemetry data with correct ordering and naming, including random variations."""
        # Convert temperature from Kelvin to Celsius
        bme_temp = round(temperature - 273.15, 2)
        bme_pressure = round(pressure, 2)

        # Approximate BME altitude from pressure (barometric formula):
        # h = (1 - (P / 1013.25)^(1/5.255)) * 44330.77
        bme_altitude = round((1.0 - (bme_pressure / 1013.25)**(1/5.255)) * 44330.77, 2)
        bme_humidity = round(humidity, 2)

        if standby:
            accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
            gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
            speed = 0.0
        else:
            # Simulate accelerations and gyroscopes with scenario-based randomization
            base_accel_x = (self.thrust / self.mass) - self.gravity
            # Add a small random variance to accel_x to simulate turbulence or uneven thrust
            accel_x = round(base_accel_x + random.uniform(-0.2, 0.2), 2)
            accel_y = round(random.uniform(-0.5, 0.5), 2)
            accel_z = round(random.uniform(-0.5, 0.5), 2)

            # Gyro values with random variance
            gyro_x = round(random.uniform(80.0, 120.0), 2)
            gyro_y = round(random.uniform(-70.0, -50.0), 2)
            gyro_z = round(random.uniform(60.0, 80.0), 2)
            speed = round(self.speed, 2)

        # Simulated IMU internal temperature (not from environment)
        imu_temp = round(random.uniform(30.0, 32.0), 2)

        # GPS data (simulated fixed values)
        gps_fix = 1
        gps_fix_quality = 2
        gps_lat = 32.9394
        gps_lon = -106.922112
        gps_speed = speed
        gps_altitude = round(self.altitude, 2)
        gps_satellites = 8

        return {
            "timestamp": time_date_sync(),
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "gyro_x": gyro_x,
            "gyro_y": gyro_y,
            "gyro_z": gyro_z,
            "imu_temp": imu_temp,
            "bme_temp": bme_temp,
            "bme_pressure": bme_pressure,
            "bme_altitude": bme_altitude,
            "bme_humidity": bme_humidity,
            "gps_fix": gps_fix,
            "gps_fix_quality": gps_fix_quality,
            "gps_lat": gps_lat,
            "gps_lon": gps_lon,
            "gps_speed": gps_speed,
            "gps_altitude": gps_altitude,
            "gps_satellites": gps_satellites
        }

    def format_message(self, data, rssi, snr):
        """Formats the telemetry message following the corrected order."""
        message = (
            "[{timestamp}] {accel_x},{accel_y},{accel_z},"
            "{gyro_x},{gyro_y},{gyro_z},{imu_temp},{bme_temp},{bme_pressure},"
            "{bme_altitude},{bme_humidity},{gps_fix},{gps_fix_quality},{gps_lat},"
            "{gps_lon},{gps_speed},{gps_altitude},{gps_satellites}"
        ).format(**data)

        length = len(message)
        return (
            "$Message length: " + str(length) + "\r\n" +
            "Message: " + str(message) + "\r\n" +
            "RSSI: " + str(rssi) + "\r\n" +
            "Snr: " + "{:.2f}".format(snr) + "\r\n"
        )

# Main Program
if not serial:
    print("No serial connection available.")
    while True:
        pass

# Choose scenario: "sunny_texas" or "cold_maine"
simulation_scenario = "sunny_texas"  # try changing this to "cold_maine" and observe the changes
print("Starting rocket telemetry simulation in scenario:", simulation_scenario)
rocket_sim = RocketSimulation(scenario=simulation_scenario)
button_state = button.value

try:
    while True:
        current_button_state = button.value
        if current_button_state != button_state:
            button_state = current_button_state
            if not button_state:  # Button pressed
                if rocket_sim.state == "standby":
                    rocket_sim.state = "launch"
                    pixel.fill((255, 0, 0))  # Red for running state
                else:
                    rocket_sim.reset()
                    pixel.fill((0, 255, 0))  # Green for standby state

        if rocket_sim.state == "standby":
            data = rocket_sim.standby_mode()
        else:
            data = rocket_sim.simulate_step()

        rssi = random.randint(-100, -90)
        snr = round(random.uniform(7, 10), 2)
        formatted_message = rocket_sim.format_message(data, rssi, snr)

        serial.write(formatted_message.encode("utf-8"))
        if usb_cdc.console:
            usb_cdc.console.write(formatted_message.encode("utf-8"))

        # Small delay between simulation steps
        time.sleep(0.25)

except KeyboardInterrupt:
    print("Simulation stopped.")
