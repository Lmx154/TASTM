# Telemetry Acquisition System Testing Module

This project simulates telemetry data for a small rocket, providing realistic data for testing TAS-GUI or similar telemetry visualization tools. It includes two modes: **standby mode**, simulating the rocket resting on a landing pad, and **launch mode**, simulating a rocket launch and ascent. The simulation supports customizable environmental scenarios, each affecting the rocket's performance and sensor readings.

## Features

- **Real-time telemetry data:** Outputs data such as acceleration, altitude, pressure, temperature, and speed at a fixed simulation timestep.
- **Standby Mode:** Simulates the rocket on a landing pad with static (or minimal) variations in telemetry data.
- **Launch Mode:** Simulates a 20 kg rocket during launch, including gravitational, thrust, and drag forces, as well as changing environmental conditions with increasing altitude.
- **Button-Controlled State Switching:** A physical button toggles between standby and launch modes.
- **Scenario-Based Simulation:** Customize the simulation environment with predefined scenarios:
    - **"sunny_texas":** Hotter, drier conditions with slightly reduced drag.
    - **"cold_maine":** Colder, more humid conditions with increased drag, making ascent more challenging.
- **Adjustable Parameters:** Easily change rocket parameters (e.g., mass, thrust) or simulation aspects (e.g., delta_time) in the code to fine-tune the behavior.

## How the Simulation Works

- **Fixed Simulation Step (delta_time):** Each loop iteration simulates a fixed amount of "virtual" time (e.g., 0.25 seconds). The `time.sleep()` call in the code only controls how frequently data is sent over USB in real-time and does not affect the rocket's simulated speed or progression.
- **Environmental Factors:** Pressure, temperature, and humidity are computed based on altitude and scenario conditions. Each scenario modifies baseline environmental conditions, influencing drag and rocket performance.
- **Sensor Simulation:** Acceleration, gyroscopic, and IMU temperature readings are approximated with random variations to mimic real sensor noise and environmental turbulence.

## Hardware Requirements

![Board Image](repo/board.jpg)

- A CircuitPython-compatible microcontroller with USB CDC support.
- A breadboard with a momentary push button connected to a GPIO pin.
- Onboard NeoPixel (or external NeoPixel) for state indication (optional but supported).

## Software Requirements

- CircuitPython installed on the microcontroller.
- Required libraries included on the device: `usb_cdc`, `digitalio`, `board`, `neopixel`, `random`, `math`, `time`.

## Wiring Instructions

1. Connect a button to the microcontroller GPIO pin `D5` (or adjust in the code).
2. Ensure the correct orientation and pull-up configuration in the code (`button.pull = digitalio.Pull.UP`).

## Installation

1. Copy the `code.py` (and optionally `boot.py`) files onto the microcontroller’s CIRCUITPY drive.
2. Ensure the required CircuitPython libraries are present in the `lib` folder on the device.
3. Connect the device to your computer via USB.

## Usage

1. Connect the microcontroller to a USB port.
2. Open a serial terminal on your computer (e.g., screen, PuTTY) to observe the telemetry data.
3. By default, the device starts in standby mode (rocket on the pad).
4. Press the button to toggle between **standby** and **launch** modes.
5. To change scenarios, edit the `simulation_scenario` variable in `code.py` (e.g., set `simulation_scenario = "cold_maine"`).
6. Observe how different scenarios and modes influence the telemetry data output.

## Telemetry Data Table

| Value in Data String | Mapped Variable   | Description                                   |
|----------------------|-------------------|-----------------------------------------------|
| `0.08`               | `accel_x`         | IMU X-axis acceleration (m/s²)                |
| `-0.40`              | `accel_y`         | IMU Y-axis acceleration (m/s²)                |
| `-9.74`              | `accel_z`         | IMU Z-axis acceleration (m/s²)                |
| `14.00`              | `gyro_x`          | IMU X-axis angular velocity (°/s)             |
| `16.00`              | `gyro_y`          | IMU Y-axis angular velocity (°/s)             |
| `-96.00`             | `gyro_z`          | IMU Z-axis angular velocity (°/s)             |
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

**Standby Mode:**
```
$Message length: 115
Message: [2024/12/19 (Thursday) 13:06:42] 0.0,0.0,0.0,0.0,0.0,0.0,30.57,35.0,1013.25,0.0,20.0,1,2,32.9394,-106.922,0.0,0.0,8
RSSI: -97
Snr: 7.39
```

**Launch Mode:**
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
   - Provides methods for standby and launch simulation.

2. **Environment and Sensors:**
   - `update_environmental_factors()` calculates pressure, temperature, and humidity based on altitude and scenario.
   - `generate_data()` simulates IMU (accelerometer, gyroscope), BME280 (temperature, pressure, humidity), and GPS readings.

3. **Button Integration:**
   - Toggles rocket state between standby and launch when pressed.

4. **NeoPixel Integration:**
   - Indicates the rocket's current state (green for standby, red for launch).

## Customization

- Change the `simulation_scenario` variable in `code.py` to switch between "sunny_texas" and "cold_maine".
- Adjust `mass`, `thrust`, or `drag_coefficient` in the `RocketSimulation` class to simulate different rocket configurations.
- Modify `delta_time` to change the simulation timestep and observe the effect on altitude and speed calculations.