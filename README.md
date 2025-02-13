# 🤖 Robot Differential Drive RPM Simulation with IPC

Welcome to the **Robot Differential Drive RPM Simulation with IPC**  project! 🚀 This repository contains a set of ROS 2-based programs that calculate and simulate wheel RPM for a differential drive robot, leveraging Inter-Process Communication (IPC) for data exchange.

## 📌 Features

✅ Computes **wheel RPM** from `cmd_vel` (linear & angular velocity).⏲✅ Implements **Inter-Process Communication (IPC)** for seamless data sharing.⚡✅ Provides **real-time visualization** using Python plotting to analyze robot behavior.📈✅ Data Logging: **Logs RPM data** and related parameters for analysis.💻

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

## 🚀 Installation & Setup
### 🔹 Prerequisites
Make sure you have the following installed:
- **ROS 2 (Humble)** 🤖
- **C++17 & Python3** 🛠️
- **Colcon Build System** ⚙️
- **Matplotlib, Seaborn (for visualization)** 📊

### 🔹 Setup Instructions
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

### 🔹 Usage
### 1. Launch the System
To run all components together:
```bash
ros2 launch rse_assignment launch_all.py
```
This will:
1. **Play a ROS bag file** with recorded motion data.🎥
2. **Run script_a** to calculate RPM values from `cmd_vel`.🏎️
3. **Run script_b** to handle IPC-based data exchange.🔄
4. **Run script_c** to visualize the data on Restful API (Dash).📊
5. **Run seaborn.py** to visualize the data on seaborn GUI.📈

### 2. Running Components Individually
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

## 🛠️ Future Improvements
✨ Add support for **WebSockets-based IPC** for cloud-based monitoring 🌐  
✨ Implement **robot control via Joystick/Gamepad** 🎮  
✨ Optimize performance for **real-time execution** ⏳

## 👨‍💻 Contributing
Pull requests are welcome! 🎉 If you have ideas for improvements, feel free to fork and submit PRs.

## 📞 Contact
📧 Email: ybbhaskar19@gmail.com  
🐙 GitHub: https://github.com/yashbhaskar
