# SCARA Robot Text Drawing Simulator

This project implements a SCARA robot simulator that can draw text using MuJoCo physics engine. The robot follows paths defined in DXF files to write text on a virtual surface.

![image](https://github.com/user-attachments/assets/b9112695-fa58-4438-941c-b78862ea359c)


⚠️ **Note: This project is currently in development. Path following has known issues and character support is limited.**

## Current Status

- Basic SCARA robot simulation works
- Inverse kinematics implemented
- Basic visualization working
- Path following needs improvement
- Character drawing not yet reliable

## Features

- 3-DOF SCARA robot simulation
- Text to DXF conversion (limited characters)
- Real-time visualization of robot movement
- Analytical inverse kinematics
- Path interpolation (needs improvement)

## Prerequisites

- Python 3.8 or higher
- MuJoCo 3.0.0 or higher
- NumPy
- ezDXF

## Installation

1. Create a virtual environment:
```bash
python -m venv scara_env
```

2. Activate the virtual environment:
```bash
# On Windows:
scara_env\Scripts\activate
# On Unix or MacOS:
source scara_env/bin/activate
```

3. Install required packages:
```bash
pip install -r requirements.txt
```

## Project Structure

```
scara_robot/
├── main.py              # Main execution script
├── robot_controller.py  # SCARA robot IK and control
├── visualization.py     # Visualization and tracing
├── path_manager.py     # Path following logic
├── dxf_parser.py       # DXF file handling
├── create_text_dxf.py  # Text to DXF conversion
├── scara_robot.xml     # Robot model definition
├── requirements.txt    # Project dependencies
└── README.md          # This file
```

## Usage

1. Basic execution:
```bash
python main.py
```

2. Create custom text:
```python
from create_text_dxf import create_text_dxf
create_text_dxf("HI", height=15)
```

## Configuration

Key parameters in `main.py`:
```python
PEN_UP_HEIGHT = 0.05      # Height when not drawing
PEN_DOWN_HEIGHT = 0.0     # Height when drawing
INTERPOLATION_POINTS = 10  # Points between vertices
DISTANCE_THRESHOLD = 0.01  # Target reaching threshold
```

## Known Issues

1. Path Following:
   - Inconsistent movement between points
   - Pen up/down movements need refinement
   - Transition between paths needs improvement

2. Character Support:
   - Currently no reliable character support
   - DXF generation needs improvement
   - Path interpolation needs work

## Development Roadmap

1. Fix path following issues:
   - Implement proper trajectory planning
   - Improve pen movement control
   - Add smooth transitions

2. Implement character support:
   - Add reliable character generation
   - Improve path interpolation
   - Add more characters

## Debugging

To debug path following issues:
1. Check console output for IK solutions
2. Watch the green (target) and blue (current) markers
3. Monitor the red trace points
4. Adjust control parameters in `scara_robot.xml`

## Robot Specifications

- Link lengths: 0.4m each
- Workspace radius: ~0.8m
- Joint ranges:
  - Joint 1: ±π radians (base rotation)
  - Joint 2: ±π radians (elbow)
  - Joint 3: -0.2m to 0m (vertical)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request


