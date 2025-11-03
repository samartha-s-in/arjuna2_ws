# Arjuna ROS 2 Package

TODO: Add package description

## Package Structure

This package contains the following modules:
- **Arjuna_arduino**: Arduino communication scripts
- **Arjuna_opencv**: Computer vision and camera processing
- **Arjuna_navigation**: Navigation and path planning
- **Arjuna_voice**: Speech recognition and voice control
- **Arjuna_web**: Web interface integration
- **Arjuna_cpu_ram**: System monitoring
- **STservo_sdk**: Servo motor SDK
- **Newrro_NavLib**: Navigation library

## Installation

Build the package using colcon:

```bash
cd ~/arjuna2_ws
colcon build --packages-select arjuna
source install/setup.bash
```

## Usage

Launch files are available in the `launch/` directory.

