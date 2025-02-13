# 🤖 Robot Differential Drive RPM Simulation with IPC

Welcome to the **Robot Differential Drive RPM Simulation with IPC**  project! 🚀 This repository contains a set of ROS 2-based programs that calculate and simulate wheel RPM for a differential drive robot, leveraging Inter-Process Communication (IPC) for data exchange.
This project simulates the RPM (Revolutions Per Minute) control of a differential drive robot using real-time data exchange and inter-process communication (IPC). Built using ROS2, it implements a system for controlling and monitoring wheel RPMs, handling differential drive kinematics, and exchanging data between processes without relying on ROS nodes.

---

## 📌 Features
✅ Computes **wheel RPM** from `cmd_vel` (linear & angular velocity).⏲  
✅ Implements **Inter-Process Communication (IPC)** for seamless data sharing.⚡  
✅ Provides **real-time visualization** using Python plotting to analyze robot behavior.📈  
✅ Data Logging: **Logs RPM data** and related parameters for analysis.💻  

---

## 📂 Project Structure
```
rse_assignment/
│── CMakeLists.txt
│── package.xml
│── launch/
│   └── launch_all.py  # Launch all files
│── src/
│   ├── script_a.cpp   # RPM calculation from cmd_vel
│   ├── script_b.cpp   # IPC data exchange
│── scripts/
│   └── script_c.py    # Visualization through Restful API (Dash)
│   └── seaborn.py     # Visualization through Seaborn
│── bag_files/
│   └── rse_assignment.db3
│── include/
│   ├── rse_assignment
│   ├── Httplib
│       └── httplib.h
│── README.md
```

---

## 🚀 Installation & Setup
### 🔹 Prerequisites
Make sure you have the following installed:
- **ROS 2 (Humble)** 🤖
- **C++17 & Python3** 🛠️
- **Colcon Build System** ⚙️
- **Matplotlib, Seaborn (for visualization)** 📊
- **Httplib (C++ library)** 📊
- **Requests, Dash, Plotly, Numpy (Python Libraries)** 📊

### 🔹 Setup Instructions
## ✅ Install ROS Dependencies
Install ROS 2 (if not already installed)
```bash
sudo apt update && sudo apt install -y ros-humble-desktop
```
Source ROS 2 setup file
```bash
source /opt/ros/humble/setup.bash
```
Create a ROS 2 workspace
```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
```
Clone the repository
```bash
git clone https://github.com/yourusername/robot-differential-drive-rpm-ipc.git rse_assignment
```
Navigate to the workspace
```bash
cd ~/ros_ws/
```
Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```
Build the package
```bash
colcon build --packages-select rse_assignment
```
Source the workspace
```bash
source install/setup.bash
```
## ✅ Install C++ Dependencies
Install C++ Compiler and Build Tools:
```bash
sudo apt update && sudo apt install -y build-essential cmake g++ gcc
```
Run the following command to install necessary C++ libraries:
```bash
sudo apt update && sudo apt install -y libhttplib-dev libjsoncpp-dev libboost-all-dev cmake
```
If libhttplib-dev is not available in your package manager, install it manually:
```bash
git clone https://github.com/yhirose/cpp-httplib.git
cd cpp-httplib
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
## ✅ Install Python Dependencies
Install Python and Pip:
```bash
sudo apt install -y python3 python3-pip python3-venv
```
Use pip to install the required Python packages:
```bash
pip install requests dash plotly numpy matplotlib
```

---

### 🔹 Usage
### 1. Launch the System
To run all components together:
```bash
ros2 launch rse_assignment launch_all.py
```
https://github.com/user-attachments/assets/212c8b2c-a5a5-45a0-84e4-4ce8347943e6
This will:
1. **Play a ROS bag file** with recorded motion data.🎥
2. **Run script_a** to calculate RPM values from `cmd_vel`.🏎️
3. **Run script_b** to handle IPC-based data exchange.🔄
4. **Run script_c** to visualize the data on Restful API (Dash).📊
5. **Run seaborn.py** to visualize the data on seaborn GUI.📈

### 2. Running Components Individually
https://github.com/user-attachments/assets/332b1370-0dd7-4a27-a6f4-3c308d3a7bbc
- **Play Bag File**
  ```bash
  ros2 bag play ~/ros_ws/src/rse_assignment/bag_files/rse_assignment.db3
  ```
- **Run RPM Calculation**
  ```bash
  ros2 run rse_assignment script_a
  ```
- **Run IPC Data Exchange**
  ```bash
  ros2 run rse_assignment script_b
  ```
- **Run Visualization on Web through dash**
  ```bash
  ros2 run rse_assignment script_c.py
  ```
- **Run Visualization on GUI through seaborn**
  ```bash
  ros2 run rse_assignment seaborn.py
  ```

---

## 🔄 Inter-Process Communication (IPC)
This project implements **IPC without ROS 2 topics**, using **shared memory (Boost IPC)** or **named pipes** for efficient data transfer between processes.

| Component        | Role 📌 |
|-----------------|---------|
| `script_a.cpp`  | Reads `cmd_vel` and computes wheel RPM ⚙️ |
| `script_b.cpp`  | Facilitates IPC for data transfer 🔄 |
| `script_c.py`   | Plots RPM data in real time 📊 |

- **Shared Memory** for high-speed data sharing.
- **Named Pipes (FIFO)** for process synchronization.
- **Sockets (TCP/UDP/WebSockets)** for networked communication.

---

## Visualization
The `script_c.py` script generates real-time plots:
- Wheel RPM vs. Time
- Velocity Commands vs. RPM

---

## 🛠️ Future Improvements
✨ Add support for **WebSockets-based IPC** for cloud-based monitoring 🌐  
✨ Implement **robot control via Joystick/Gamepad** 🎮  
✨ Optimize performance for **real-time execution** ⏳

---

## 👨‍💻 Contributing
Pull requests are welcome! 🎉 If you have ideas for improvements, feel free to fork and submit PRs.

---

## 📞 Contact
📧 Email: ybbhaskar19@gmail.com  
🐙 GitHub: https://github.com/yashbhaskar
