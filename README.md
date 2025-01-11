# Telemetry Acquisition System Testing Module

This project simulates telemetry data for a small rocket, providing realistic data for testing TAS-GUI or similar telemetry visualization tools. The simulation supports customizable environmental scenarios, each affecting the rocket's performance and sensor readings.

## Features

- **Real-time telemetry data:** Outputs data such as acceleration, altitude, pressure, temperature, and speed at a fixed simulation timestep.
- **Launch Mode:** Simulates a 20 kg rocket during launch, including gravitational, thrust, and drag forces, as well as changing environmental conditions with increasing altitude.
- **Scenario-Based Simulation:** Customize the simulation environment with predefined scenarios:
    - **"sunny_texas":** Hotter, drier conditions with slightly reduced drag.
    - **"cold_maine":** Colder, more humid conditions with increased drag, making ascent more challenging.
- **Adjustable Parameters:** Easily change rocket parameters (e.g., mass, thrust) or simulation aspects (e.g., delta_time) in the code to fine-tune the behavior.
- **Real-Time Clock Integration:** Uses DS3231 RTC module for accurate timestamp generation in telemetry data.

## How the Simulation Works

- **Fixed Simulation Step (delta_time):** Each loop iteration simulates a fixed amount of "virtual" time (e.g., 0.25 seconds). The `time.sleep()` call in the code only controls how frequently data is sent over USB in real-time and does not affect the rocket's simulated speed or progression.
- **Environmental Factors:** Pressure, temperature, and humidity are computed based on altitude and scenario conditions. Each scenario modifies baseline environmental conditions, influencing drag and rocket performance.
- **Sensor Simulation:** Acceleration, gyroscopic, and IMU temperature readings are approximated with random variations to mimic real sensor noise and environmental turbulence.

## Hardware Requirements

![Board Image](repo/board.jpg)

- A CircuitPython-compatible microcontroller with USB CDC support.
- DS3231 Real-Time Clock module connected via I2C (SCL: GP5, SDA: GP4 for RP2040)

## Software Requirements

- CircuitPython installed on the microcontroller.
- Required libraries included on the device: 
  - `usb_cdc`
  - `board`
  - `random`
  - `math`
  - `time`
  - `busio`
  - `adafruit_ds3231`

## Installation

1. Copy the `code.py` (and optionally `boot.py`) files onto the microcontroller’s CIRCUITPY drive.
2. Ensure the required CircuitPython libraries are present in the `lib` folder on the device.
3. Connect the DS3231 module to the appropriate I2C pins (default: SCL=GP5, SDA=GP4).
4. Connect the device to your computer via USB.

## Usage

1. Connect the microcontroller to a USB port.
2. Open a serial terminal on your computer (e.g., screen, PuTTY) to observe the telemetry data.
3. The simulation will automatically start and run continuously.
4. To change scenarios, edit the `simulation_scenario` variable in `code.py` (e.g., set `simulation_scenario = "cold_maine"`).
5. Observe how different scenarios influence the telemetry data output.

## Time Synchronization

The system automatically synchronizes with the DS3231 RTC module on startup. Each telemetry message includes an accurate timestamp in the format:

## Telemetry Data Table

| Value in Data String | Mapped Variable   | Description                                   |
|----------------------|-------------------|-----------------------------------------------|
| `0.08`               | `accel_x`         | IMU X-axis acceleration (m/s²)                |
| `-0.40`              | `accel_y`         | IMU Y-axis acceleration (m/s²)                |
| `-9.74`              | `accel_z`         | IMU Z-axis acceleration (m/s²)                |
| `14.00`              | `gyro_x`          | IMU X-axis angular velocity (°/s)             |
| `16.00`              | `gyro_y`          | IMU Y-axis angular velocity (°/s)             |
| `-96.00`             | `gyro_z`          | IMU Z-axis angular velocity (°/s)             |
| `14.00`              | `gyro_x`          | IMU X-axis accumulated angle (°)              |
| `16.00`              | `gyro_y`          | IMU Y-axis accumulated angle (°)              |
| `-96.00`             | `gyro_z`          | IMU Z-axis accumulated angle (°)              |
| `31.25`              | `imu_temp`        | IMU internal temperature (°C)                 |
| `33.56`              | `bme_temp`        | BME280 temperature reading (°C)               |
| `1009.91`            | `bme_pressure`    | BME280 atmospheric pressure (hPa)             |
| `-31.06`             | `bme_altitude`    | BME280 altitude (m)                           |
| `35.39`              | `bme_humidity`    | BME280 humidity (%)                           |
| `1`                  | `gps_fix`         | GPS fix status (1 = fixed, 0 = not fixed)      |
| `2`                  | `gps_fix_quality` | GPS fix quality (e.g., 1 = GPS fix, 2 = DGPS) |
| `26.273800`          | `gps_lat`         | GPS latitude (decimal degrees)                 |
| `-98.431976`         | `gps_lon`         | GPS longitude (decimal degrees)                |
| `0.16`               | `gps_speed`       | GPS ground speed (m/s)                        |
| `68.00`              | `gps_altitude`    | GPS altitude (m)                              |
| `8`                  | `gps_satellites`  | Number of GPS satellites in use               |

## Telemetry Data Format

Data is output in the following format:

```
[timestamp] accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, imu_temp, bme_temp, bme_pressure, bme_altitude, bme_humidity, gps_fix, gps_fix_quality, gps_lat, gps_lon, gps_speed, gps_altitude, gps_satellites
```

### Example Output

```
$Message length: 130
Message: [2024/12/19 (Thursday) 13:00:02] 15.1,-0.02,-0.4,82.33,-65.0,72.93,31.17,7.52,616.2,4003.17,0,1,2,32.9394,-106.922,82.16,4227.45,8
RSSI: -97
Snr: 7.87
```

## Code Overview

### Main Components

1. **`RocketSimulation` Class:**
   - Manages rocket state (speed, altitude, mass, thrust, gravity, and drag).
   - Integrates scenario conditions affecting temperature, humidity, and drag.
   - Provides methods for launch simulation.

2. **Environment and Sensors:**
   - `update_environmental_factors()` calculates pressure, temperature, and humidity based on altitude and scenario.
   - `generate_data()` simulates IMU (accelerometer, gyroscope), BME280 (temperature, pressure, humidity), and GPS readings.

## Customization

- Change the `simulation_scenario` variable in `code.py` to switch between "sunny_texas" and "cold_maine".
- Adjust `mass`, `thrust`, or `drag_coefficient` in the `RocketSimulation` class to simulate different rocket configurations.
- Modify `delta_time` to change the simulation timestep and observe the effect on altitude and speed calculations.