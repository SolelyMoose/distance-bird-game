# Distance Bird Game

**Updraft**: steer a bird through a winding canyon by moving your hand over an ultrasonic distance sensor.

## Hardware
- Arduino Uno
- HC-SR04 ultrasonic sensor (Trig → pin 7, Echo → pin 8, VCC → 5V, GND → GND)

## Setup
1. Open this folder in VS Code with PlatformIO and upload the sketch (`src/main.cpp`) to the Uno.
2. Install the Python dependencies: `pip install pygame pyserial`
3. Close any serial monitor, then run:
   ```
   cd game
   python updraft.py
   ```

## Controls
- Hand height over the sensor sets the bird's height
- **L** / **H**: set the current distance as the low / high end of the range
- **SPACE**: start or restart
- **M**: toggle mouse control
- **ESC**: quit

Options: `--port COM3`, `--low 5`, `--high 60` (distances in cm).
