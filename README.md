# Robot Differential Drive RPM & IPC Simulation

## Overview
This project focuses on the simulation of a **differential drive robot** where wheel **RPM calculations** are performed based on velocity commands (`cmd_vel`). The system also implements **Inter-Process Communication (IPC)** to share data across different processes, enabling real-time analysis and visualization.

## Features
- **Differential Drive RPM Calculation**: Computes the left and right wheel RPM from velocity commands.
- **Inter-Process Communication (IPC)**: Facilitates data exchange between multiple processes without using ROS.
- **Simulation & Visualization**: Plots real-time data to analyze robot behavior.
- **Data Logging**: Logs RPM data and related parameters for analysis.
- **Bag File Playback**: Uses `ros2 bag play` for replaying sensor data.

## Project Structure
```
rse_assignment/
│── CMakeLists.txt
│── package.xml
│── launch/
│   └── launch_all.py
│── src/
│   ├── script_a.cpp   # RPM calculation from cmd_vel
│   ├── script_b.cpp   # IPC data exchange
│── scripts/
│   └── script_c.py    # Visualization & logging
│── bag_files/
│   └── rse_assignment_unbox_robotics.db3
│── include/rse_assignment/
```

## Installation
### Prerequisites
- **ROS 2 (Humble/Foxy)**
- **C++17 & Python3**
- **Colcon Build System**
- **Matplotlib (for visualization)**

### Setup Instructions
```bash
# Clone the repository into your ROS workspace
cd ~/ros_ws/src
git clone https://github.com/yashbhaskar/robot-differential-drive-rpm-ipc-simulation.git

# Navigate to the workspace root
cd ~/ros_ws

# Build the package
colcon build --packages-select rse_assignment

# Source the workspace
source install/setup.bash
```

## Usage
### 1. Launch the System
To run all components together:
```bash
ros2 launch rse_assignment launch_all.py
```
This will:
1. **Play a ROS bag file** with recorded motion data.
2. **Run script_a** to calculate RPM values from `cmd_vel`.
3. **Run script_b** to handle IPC-based data exchange.
4. **Run script_c** to visualize the data.

### 2. Running Components Individually
- **Run RPM Calculation**
  ```bash
  ros2 run rse_assignment script_a
  ```
- **Run IPC Data Exchange**
  ```bash
  ros2 run rse_assignment script_b
  ```
- **Run Visualization**
  ```bash
  ros2 run rse_assignment script_c.py
  ```
- **Replay Bag File**
  ```bash
  ros2 bag play ~/ros_ws/src/rse_assignment/bag_files/rse_assignment_unbox_robotics.db3
  ```

## Inter-Process Communication (IPC)
This project does not rely on ROS topics for communication between `script_a`, `script_b`, and `script_c`. Instead, it utilizes:
- **Shared Memory** for high-speed data sharing.
- **Named Pipes (FIFO)** for process synchronization.
- **Sockets (TCP/UDP/WebSockets)** for networked communication.

## Visualization
The `script_c.py` script generates real-time plots:
- Wheel RPM vs. Time
- Velocity Commands vs. RPM

## Future Improvements
- Add **WebSockets for remote monitoring**.
- Implement **PID control for better RPM accuracy**.
- Support **hardware integration with real robots**.

## Contributing
Feel free to open issues and submit pull requests. Contributions are welcome!

## License
This project is licensed under the **MIT License**.

## Author
**Your Name** - [GitHub Profile](https://github.com/your-username)

