import time
import usb_cdc
import random
import math
import board
import digitalio
import neopixel
import busio
from adafruit_ds3231 import DS3231

# I2C setup for DS3231
# Replace GP4 and GP5 with your board's SDA and SCL pins
# Example: Adafruit Feather RP2040 uses board.SDA and board.SCL
# i2c = busio.I2C(board.SDA, board.SCL) # Adafruit Feather RP2040
i2c = busio.I2C(board.GP27, board.GP26)  # RP Pi Pico 2 RP2040
rtc = DS3231(i2c)

# USB CDC setup
serial = usb_cdc.data

# Button setup
# Replace GP5 with your board's GPIO pin
# button = digitalio.DigitalInOut(board.D5)  # Adafruit Feather RP2040
button = digitalio.DigitalInOut(board.GP5)  # RP Pi Pico 2 RP2040
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP  # Use pull-up resistor

# NeoPixel setup
# Replace NEOPIXEL and brightness settings based on your board
# pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)  # Adafruit Feather RP2040 NeoPixel setup
# pixel.brightness = 0.2  # Set brightness (0.0 to 1.0)
PIXEL_PIN = board.GP0  # RP Pi Pico 2 RP2040 NeoPixel setup
NUM_PIXELS = 8
BRIGHTNESS = 0.5
pixel = neopixel.NeoPixel(PIXEL_PIN, NUM_PIXELS, brightness=BRIGHTNESS, auto_write=False)

def time_date_sync():
    """Fetches the current time and date from the DS3231 RTC."""
    now = rtc.datetime  # Fetch time from DS3231
    year = now.tm_year
    month = now.tm_mon
    day = now.tm_mday
    weekday = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"][now.tm_wday]
    hours = now.tm_hour
    minutes = now.tm_min
    seconds = now.tm_sec
    return f"{year}/{month:02}/{day:02} ({weekday}) {hours:02}:{minutes:02}:{seconds:02}"

# Example function to set the RTC time (run once to initialize RTC)
def set_rtc_time():
    """Sets the DS3231 RTC to the current system time."""
    rtc.datetime = time.struct_time((2024, 12, 21, 15, 0, 0, 5, -1, -1))  # Adjust to desired initial time

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
        self.base_temp = 288.15  # Baseline temperature in Kelvin at sea level (15°C)
        self.base_humidity = 50.0
        self.drag_coefficient = 0.05
        self.delta_time = 0.25
        self.state = "standby"
        self.apply_scenario_conditions()

    def apply_scenario_conditions(self):
        """Adjust environment and drag based on scenario."""
        if self.scenario == "sunny_texas":
            self.base_temp = 308.15
            self.base_humidity = 20.0
            self.drag_coefficient = 0.045
        elif self.scenario == "cold_maine":
            self.base_temp = 263.15
            self.base_humidity = 80.0
            self.drag_coefficient = 0.055
        else:
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
        acceleration = (self.thrust - drag - self.gravity * self.mass) / self.mass
        return self.speed + acceleration * self.delta_time

    def update_altitude(self):
        return self.altitude + self.speed * self.delta_time

    def update_environmental_factors(self, altitude=None):
        if altitude is None:
            altitude = self.altitude
        sea_level_pressure = 1013.25  # hPa
        scale_height = 8500  # meters
        pressure = sea_level_pressure * math.exp(-altitude / scale_height)
        lapse_rate = -0.0065  # K/m
        temperature = self.base_temp + lapse_rate * altitude
        temperature = max(temperature, 216.65)
        humidity = max(0, self.base_humidity - (altitude / 200))
        return pressure, temperature, humidity

    def generate_data(self, pressure, temperature, humidity, standby=False):
        bme_temp = round(temperature - 273.15, 2)
        bme_pressure = round(pressure, 2)
        bme_altitude = round((1.0 - (bme_pressure / 1013.25)**(1/5.255)) * 44330.77, 2)
        bme_humidity = round(humidity, 2)

        if standby:
            accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
            gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
            speed = 0.0
        else:
            base_accel_x = (self.thrust / self.mass) - self.gravity
            accel_x = round(base_accel_x + random.uniform(-0.2, 0.2), 2)
            accel_y = round(random.uniform(-0.5, 0.5), 2)
            accel_z = round(random.uniform(-0.5, 0.5), 2)
            gyro_x = round(random.uniform(80.0, 120.0), 2)
            gyro_y = round(random.uniform(-70.0, -50.0), 2)
            gyro_z = round(random.uniform(60.0, 80.0), 2)
            speed = round(self.speed, 2)

        imu_temp = round(random.uniform(30.0, 32.0), 2)
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

# Uncomment this to set RTC time initially, then comment it back
set_rtc_time()

simulation_scenario = "sunny_texas"
print("Starting rocket telemetry simulation with DS3231 RTC support.")
rocket_sim = RocketSimulation(scenario=simulation_scenario)
button_state = button.value

try:
    while True:
        current_button_state = button.value
        if current_button_state != button_state:
            button_state = current_button_state
            if not button_state:
                if rocket_sim.state == "standby":
                    rocket_sim.state = "launch"
                    pixel.fill((255, 0, 0))
                    pixel.show()
                else:
                    rocket_sim.reset()
                    pixel.fill((0, 255, 0))
                    pixel.show()

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

        time.sleep(0.25)

except KeyboardInterrupt:
    print("Simulation stopped.")

