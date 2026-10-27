# Distance Bird Game

**Updraft**: steer a bird through a winding canyon by moving your hand over a TF-Luna lidar and an ultrasonic sensor. The game combines both readings: the ultrasonic's wide beam catches your hand, and the TF-Luna adds precision.

## Hardware
- Arduino Uno
- TF-Luna lidar in I2C mode (SDA → A4, SCL → A5, 5V, GND)
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

Options: `--port COM3`, `--low 50`, `--high 100` (distances in cm).
