# Rocket Telemetry Simulation

This project simulates telemetry data for a small rocket, providing realistic data for testing TAS-GUI. It includes two modes: **standby mode**, simulating the rocket on a landing pad, and **launch mode**, simulating a rocket launch and ascent.

## Features

- **Real-time telemetry data:** Outputs data such as acceleration, altitude, pressure, temperature, and speed.
- **Standby Mode:** Simulates the rocket on a landing pad.
- **Launch Mode:** Simulates a 20 kg rocket during launch, including environmental effects.
- **Button-controlled state switching:** A physical button toggles between standby and launch modes.

## Hardware Requirements

- A microcontroller with USB CDC support.
- A breadboard with a button connected to a GPIO pin.
- CircuitPython-compatible microcontroller.

## Software Requirements

- CircuitPython installed on the microcontroller.
- Libraries: `usb_cdc`, `digitalio`, and `board`.

## Wiring Instructions

1. Connect a button to the microcontroller GPIO pin `D5` (or adjust in the code).

## Installation

1. Copy the `code.py` and `boot.py` files onto the microcontroller.
2. Ensure the required CircuitPython libraries are present on the device.
3. Connect the device to your computer via USB.

## Usage

1. Connect the microcontroller to a USB port.
2. Open a serial terminal to observe telemetry data.
3. Press the button to toggle between **standby** and **launch** modes.

## Telemetry Data Format

Data is output in the following format:

```
YYYY|MM|DD|Weekday|HH:MM:SS|accel_x|accel_y|accel_z|gx|gy|gz|temp_c|temp_f|pressure|altitude|humidity|fix|fix_quality|latitude|longitude|speed|altitude|satellites
```

### Example Output

**Standby Mode:**
```
2024|12|13|Friday|16:43:39|0.0|0.0|0.0|0.0|0.0|0.0|15.0|59.0|1013.25|0.0|50.0|1.0|2|32.9394|-106.922|0.0|0.0|8
```

**Launch Mode:**
```
2024|12|13|Friday|16:43:39|0.0|0.1|0.2|90.0|-50.0|60.0|14.8|58.9|1012.0|100.5|49.0|1.0|2|32.9394|-106.922|5.0|100.5|8
```

## Code Overview

### Main Components

1. **`RocketSimulation` Class:**
   - Handles rocket state, including speed, altitude, and environmental factors.
   - Includes methods for standby and launch simulations.

2. **`update_speed`, `update_altitude`, `update_environmental_factors` Functions:**
   - Calculates speed, altitude, and environmental parameters dynamically based on the rocket's state.

3. **Button Integration:**
   - Toggles the rocket's state between standby and launch using a button input.

## Customization

- Adjust rocket parameters (e.g., mass, thrust) by modifying the `RocketSimulation` class.
- Change the button GPIO pin in the `button = digitalio.DigitalInOut(board.D5)` line.


---


