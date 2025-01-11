import time
import usb_cdc
import random
import math
import board
import busio
from adafruit_ds3231 import DS3231

# I2C setup for DS3231
i2c = busio.I2C(board.GP5, board.GP4)  # RP Pi Pico 2 RP2040 (CHANGE IF USING ANOTHER BOARD)
rtc = DS3231(i2c)

# USB CDC setup (console only, single serial)
serial = usb_cdc.console

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

def set_rtc_time():
    """
    Sets the DS3231 RTC to the board's current local time.
    NOTE: On many CircuitPython boards, time.localtime() is just
    'time since power-on' unless you've otherwise set it,
    or have an internet-capable board doing NTP sync.
    """
    board_time = time.localtime()
    # Create a time.struct_time for the RTC
    rtc.datetime = time.struct_time(
        (
            board_time.tm_year,
            board_time.tm_mon,
            board_time.tm_mday,
            board_time.tm_hour,
            board_time.tm_min,
            board_time.tm_sec,
            board_time.tm_wday,
            board_time.tm_yday,
            board_time.tm_isdst,
        )
    )
    print("RTC set to:", time_date_sync())

class RocketSimulation:
    def __init__(self, scenario="sunny_texas"):
        self.scenario = scenario
        self.reset()

    def reset(self):
        """Resets the rocket to its initial state and applies scenario conditions."""
        self.altitude = 0.0
        self.speed = 0.0
        self.mass = 20.0
        self.thrust = 500.0
        self.gravity = 9.81
        self.base_temp = 288.15  # Baseline temperature in Kelvin at sea level (15°C)
        self.base_humidity = 50.0
        self.drag_coefficient = 0.05
        self.delta_time = 0.25

        # NEW: Orientation angles (accumulating gyro).
        # We'll treat our random 'gyro_x/y/z' as degrees/s and accumulate them.
        self.gx_angle = 0.0
        self.gy_angle = 0.0
        self.gz_angle = 0.0

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
        drag = self.drag_coefficient * self.speed**2
        self.speed = self.update_speed(drag)
        self.altitude = self.update_altitude()
        pressure, temperature, humidity = self.update_environmental_factors()
        data = self.generate_data(pressure, temperature, humidity)
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
        # Clamp to a minimum of 216.65 K
        temperature = max(temperature, 216.65)
        # Decrease humidity with altitude
        humidity = max(0, self.base_humidity - (altitude / 200))
        return pressure, temperature, humidity

    def generate_data(self, pressure, temperature, humidity):
        # BME280-like simulation
        bme_temp = round(temperature - 273.15, 2)
        bme_pressure = round(pressure, 2)
        bme_altitude = round(
            (1.0 - (bme_pressure / 1013.25) ** (1 / 5.255)) * 44330.77, 2
        )
        bme_humidity = round(humidity, 2)

        # Simulate accelerometer
        base_accel_x = (self.thrust / self.mass) - self.gravity
        accel_x = round(base_accel_x + random.uniform(-0.2, 0.2), 2)
        accel_y = round(random.uniform(-0.5, 0.5), 2)
        accel_z = round(random.uniform(-0.5, 0.5), 2)

        # Simulate gyro rates (degrees/s)
        gyro_x_rate = round(random.uniform(80.0, 120.0), 2)
        gyro_y_rate = round(random.uniform(-70.0, -50.0), 2)
        gyro_z_rate = round(random.uniform(60.0, 80.0), 2)

        # --- NEW: Accumulate orientation angles ---
        # Interpret these gyro rates as degrees/s over delta_time
        self.gx_angle += gyro_x_rate * self.delta_time
        self.gy_angle += gyro_y_rate * self.delta_time
        self.gz_angle += gyro_z_rate * self.delta_time

        speed = round(self.speed, 2)
        imu_temp = round(random.uniform(30.0, 32.0), 2)

        # Simulate a fixed "valid" GPS reading
        gps_fix = 1
        gps_fix_quality = 2
        gps_lat = 32.9394
        gps_lon = -106.922112
        gps_speed = speed
        gps_altitude = round(self.altitude, 2)
        gps_satellites = 8

        # Return a dictionary of the data
        # We now store BOTH the "raw" gyro rate and the "accumulated" angles.
        return {
            "timestamp": time_date_sync(),
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "gyro_x": round(self.gx_angle, 2),  # orientation angle X
            "gyro_y": round(self.gy_angle, 2),  # orientation angle Y
            "gyro_z": round(self.gz_angle, 2),  # orientation angle Z
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
            "gps_satellites": gps_satellites,
        }

    def format_message(self, data, rssi, snr):
        """
        Formats a string in the same style as the C code does:
        [timestamp] accel_x,accel_y,accel_z, gyro_x,gyro_y,gyro_z, ...
        """
        message = (
            "[{timestamp}] {accel_x},{accel_y},{accel_z},"
            "{gyro_x},{gyro_y},{gyro_z},{imu_temp},{bme_temp},{bme_pressure},"
            "{bme_altitude},{bme_humidity},{gps_fix},{gps_fix_quality},{gps_lat},"
            "{gps_lon},{gps_speed},{gps_altitude},{gps_satellites}"
        ).format(**data)

        length = len(message)
        return (
            "$Message length: " + str(length) + "\r\n"
            + "Message: " + str(message) + "\r\n"
            + "RSSI: " + str(rssi) + "\r\n"
            + "Snr: " + "{:.2f}".format(snr) + "\r\n"
        )

# Main Program
if not serial:
    print("No serial connection available.")
    while True:
        pass

# Set RTC once on startup (optional)
set_rtc_time()

simulation_scenario = "sunny_texas"
print("Starting rocket telemetry simulation with DS3231 RTC support.")
rocket_sim = RocketSimulation(scenario=simulation_scenario)

try:
    while True:
        data = rocket_sim.simulate_step()

        # Generate some dummy radio info
        rssi = random.randint(-100, -90)
        snr = round(random.uniform(7, 10), 2)
        formatted_message = rocket_sim.format_message(data, rssi, snr)

        # Output via USB serial
        serial.write(formatted_message.encode("utf-8"))
        # Also optionally show in REPL if you like
        if usb_cdc.console:
            usb_cdc.console.write(formatted_message.encode("utf-8"))

        # Wait between steps
        time.sleep(0.25)

except KeyboardInterrupt:
    print("Simulation stopped.")

