

# Mission Planner Project

This project is a Mission Planner system for drone management, integrating MAVLink communications, UDP interfaces for various peripherals, and a GUI for monitoring and interactive configuration editing.

## Features

- Modular design with separate classes for configuration, state management, and interfaces.
- GUI for real-time monitoring and interactive configuration editing.
- Supports USB and UDP joystick input with exponential adjustment.

## Setup

1. Install dependencies using:
   ```
   pip install -r requirements.txt
   ```
2. Ensure the `config.xml` file is properly configured and validated against `config.xsd`.
3. Run the application with:
   ```
   python src/main.py
   ```

## Directory Structure

See the structure above.
