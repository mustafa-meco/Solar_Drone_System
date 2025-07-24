# Solar Drone System

A comprehensive autonomous drone system built with ROS (Robot Operating System) that provides camera streaming, mission planning, and remote control capabilities.

## Features

- **Autonomous Flight Control**: Integration with PX4/MAVROS for autonomous drone operations
- **Real-time Camera Streaming**: Live video feed transmission between drone and ground station
- **Mission Planning**: Support for waypoint-based autonomous missions
- **Remote Monitoring**: GUI-based control and monitoring system
- **Distributed Architecture**: Support for multi-machine deployment (main control unit + Raspberry Pi)
- **Frame Recording**: Automatic saving of camera frames for later analysis

## System Architecture

The system consists of several ROS nodes distributed across two main platforms:

![ROS System Diagram](ROS%20System%20Diagram.png)

*Figure 1: Complete system architecture showing the distributed ROS nodes across the Control Unit (CU), Raspberry Pi, and their interconnections via WiFi/Feed links.*

### Main Control Unit (Ground Station)
- **GUI Node**: Tkinter-based interface for system control and monitoring
- **Camera Reader Node**: Displays live video feed from drone
- **Frames Saving Node**: Records and saves camera frames locally

### Drone (Raspberry Pi)
- **Camera Adapter Node**: Captures and streams camera feed
- **MAVROS Interface**: Communication with flight controller
- **Mission Execution**: Autonomous waypoint navigation

## Prerequisites

### Hardware Requirements
- Drone platform with PX4-compatible flight controller
- Raspberry Pi (for onboard processing)
- Camera module
- Ground station computer
- Network connectivity (WiFi/Radio)

### Software Requirements
- Ubuntu 18.04/20.04 with ROS Melodic/Noetic
- Python 3.6+
- OpenCV
- MAVROS
- PX4 SITL (for simulation)

## Installation

1. **Clone the repository**:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url> Solar_Drone_System
   ```

2. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-cv-bridge
   sudo apt install python3-opencv python3-numpy python3-tk python3-paramiko
   ```

3. **Install MAVROS GeographicLib datasets**:
   ```bash
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo bash ./install_geographiclib_datasets.sh
   ```

4. **Build the package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Configuration

1. **Copy and configure settings**:
   ```bash
   cp settings\ copy.json settings.json
   ```

2. **Update settings.json** with your system configuration:
   - Network IPs for main control unit and Raspberry Pi
   - SSH credentials
   - ROS node configuration

3. **Example settings.json**:
   ```json
   {
     "networking": {
       "main_cu": "192.168.1.100",
       "raspi": "192.168.1.101"
     },
     "security": {
       "main_cu": {
         "username": "your_username",
         "password": "your_password"
       },
       "raspi": {
         "username": "pi",
         "password": "raspberry"
       }
     }
   }
   ```

## Usage

### Simulation Mode

1. **Start PX4 SITL**:
   ```bash
   make px4_sitl gazebo
   ```

2. **Launch MAVROS**:
   ```bash
   roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
   ```

3. **Run drone control test**:
   ```bash
   cd ~/catkin_ws/src/Solar_Drone_System/src
   python3 ros_nodes/testingClass/bootstrap.py
   ```

### Real Hardware Deployment

1. **Start the GUI control system**:
   ```bash
   cd ~/catkin_ws/src/Solar_Drone_System/src
   python3 ros_nodes/GUI_node/bootstrap.py
   ```

2. **Use the GUI to**:
   - Start ROS master on drone
   - Launch MAVROS on drone
   - Start camera and control nodes
   - Monitor system status
   - Download recorded frames

### Available ROS Nodes

#### Local Nodes (Ground Station)
- `camera_reader_node`: Display live camera feed
- `frames_saving_node`: Save camera frames to disk
- `GUI_node`: Main control interface

#### Remote Nodes (Drone)
- `camera_adapter_node`: Camera capture and streaming
- `testingClass`: Autonomous flight control
- `Strategy`: Mission planning and execution

## Project Structure

```
Solar_Drone_System/
├── src/
│   ├── ros_nodes/
│   │   ├── GUI_node/              # Ground station GUI
│   │   ├── camera_adapter_node/   # Camera streaming
│   │   ├── camera_reader_node/    # Camera display
│   │   ├── frames_saving_node/    # Frame recording
│   │   ├── testingClass/          # Flight control
│   │   └── Strategy/              # Mission planning
│   └── lib/
│       ├── myDrone.py            # Drone control class
│       ├── mission_test.py       # Mission testing
│       ├── settings.py           # Configuration management
│       ├── ros.py                # ROS utilities
│       └── log.py                # Logging utilities
├── launch/
│   └── drone.launch              # Launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## API Reference

### MyDrone Class

The main drone control interface:

```python
from lib.myDrone import MyDrone

drone = MyDrone()
drone.takeoff(5.0)                    # Takeoff to 5m altitude
drone.set_mode('OFFBOARD')            # Switch to offboard mode
drone.handle_waypoints(waypoints)     # Navigate waypoints
drone.landing()                       # Land the drone
```

### Key Methods
- `takeoff(altitude)`: Autonomous takeoff
- `landing()`: Autonomous landing
- `set_mode(mode)`: Change flight mode
- `arming(value)`: Arm/disarm motors
- `handle_waypoints(waypoints)`: Navigate through waypoint list

## Testing

Run the mission test suite:
```bash
cd ~/catkin_ws/src/Solar_Drone_System/src
python3 ros_nodes/Strategy/bootstrap.py
```

## Troubleshooting

### Common Issues

1. **MAVROS connection failed**:
   - Check PX4 SITL is running
   - Verify fcu_url parameter
   - Ensure correct network configuration

2. **Camera not found**:
   - Check camera device permissions
   - Verify camera index in camera_adapter_node
   - Test camera with: `ls /dev/video*`

3. **SSH connection failed**:
   - Verify network connectivity
   - Check SSH credentials in settings.json
   - Ensure SSH service is running on target machine

4. **ROS nodes not communicating**:
   - Check ROS_MASTER_URI environment variable
   - Verify network configuration
   - Test with: `rostopic list`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- PX4 Development Team for the flight control stack
- MAVROS team for ROS-MAVLink integration
- OpenCV community for computer vision libraries

## Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review ROS and PX4 documentation

---

**Note**: This system is designed for research and development purposes. Always follow local regulations and safety guidelines when operating drones.
