# Coratia 3D Sonar Mapping Engine

A high-performance, real-time 3D underwater mapping, scanning, and reconstruction system developed by **Coratia Technologies** for the **Blue Robotics Ping360 scanning sonar**.

This system performs live dead-reckoning and coordinate projection by fusing range sweeps from the sonar transducer, attitude telemetry from the ROV's IMU (via Mavlink/ArduSub), and velocity telemetry from a DVL (Water Linked DVL). The output is projected into global 3D coordinates and streamed to a high-performance interactive WebGL dashboard for visual reconstruction.

---

## 🚀 Key Features

* **Real-Time 3D Reconstruction**: Visualizes sonar sweeps dynamically as a 3D point cloud, Surface mesh grid, or shaded Solid Model.
* **Real-Time 3D Reconstruction**: Visualizes sonar sweeps dynamically as a 3D point cloud, Surface mesh grid, or shaded Solid Model, utilizing local maxima peak extraction for high-fidelity edge detection.
* **Multi-Sensor EKF Fusion**:
  * **Blue Robotics Ping360**: Polar acoustic range sweeps.
  * **Mavlink Attitude Telemetry**: Auto-fallback REST API queries (`192.168.2.2:6040`) for Roll, Pitch, and Yaw if UDP is offline.
  * **Water Linked DVL Telemetry**: Velocity vectors ($v_x, v_y, v_z$) parsed from raw streams and fused with IMU using a **2D Extended Kalman Filter (EKF)**.
* **Interactive WebGL Dashboard**:
  * Sleek light-themed dashboard with orbital camera controls and 3D ROV tracking.
  * **Typeable Input Controls**: Every slider control (Threshold, Speed, IMU Yaw Offset, Sonar Offset) features a direct numeric input box for live typing or drag adjustments.
  * **Min Range Cutoff Control**: Adjustable minimum distance cutoff (`0.05 m` to `2.0 m`) to blank out ROV chassis reflections while enabling ultra-close target mapping down to 0.1m.
  * **Set Default & Reset**: "Set Default" button saves current custom settings persistently in browser `localStorage` and Python backend `settings.json` across sessions.
  * **Dynamic Controls**: The manual **Speed** slider hides automatically in Hardware Mode (since speed is DVL-driven).
  * **Scan Step Selector**: Selectable step sizes from `1g (Fine)` to `20g (Coarse)` to adjust sweep rates and resolution.
* **Dual Execution Modes**: Sandbox simulation with structured tunnel environments, or Real Hardware Mode with actual UDP/serial device connection loops.

---

## 📐 System Architecture & Signal Flow

The core coordination is handled by `mapper_3d_engine.py`, which integrates telemetry streams and forwards the computed 3D point cloud over WebSockets (port `8001`) to the HTML5 client dashboard:

```
       +-----------------------+      +-------------------+
       |  Mavlink IMU (UDP)    |      |  DVL TCP JSON     |
       |  [Roll, Pitch, Yaw]   |      |  [vx, vy, vz]     |
       +-----------+-----------+      +---------+---------+
                    |                            |
                    v                            v
              +-----+----------------------------+-----+
              |  mapper_3d_engine.py (Python)          | <---+ Ping360 Sonar (COM/UDP)
              |  - 3D Dead Reckoning (Coordinate Rot)  |      [Sweep Angle, Bins]
              |  - Polar-to-Cartesian Projection       |
              +--------------------+-------------------+
                                   | (WebSockets Port 8001)
                                   v
              +--------------------+-------------------+
              |  Web Dashboard (HTML5/Three.js)        |
              |  - Light theme visualizer              |
              |  - Real-time 3D point cloud renderer   |
              +----------------------------------------+
```

For the detailed mathematical foundations (coordinate transformations, Tait-Bryan rotation matrix integrations, and WebGL axis remappings), please refer to the detailed [Sonar 3D Mapping Logic Documentation](sonar_3d_mapping_logic.md).

---

## 🛠️ Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/VitaminDcodes/ping360.git
cd ping360
```

### 2. Environment Configuration
* **Windows (PowerShell)**: Run the automated environment setup script. This will create a Python virtual environment and install the required dependencies:
  ```powershell
  .\setup_env.ps1
  ```
* **Manual Setup (Cross-platform)**:
  ```bash
  python -m venv venv
  # Activate virtual environment
  source venv/bin/activate  # On Linux/macOS
  venv\Scripts\activate     # On Windows
  # Install packages
  pip install -r requirements.txt
  ```

---

## 💻 Usage

To launch the system, run the startup helper:

* **On Windows**: Double-click `run_mapping_system.bat` or run:
  ```powershell
  .\run_mapping_system.ps1
  ```
* **Manual launch (Python)**:
  ```bash
  python mapper_3d_engine.py [args]
  ```

### Execution Modes

#### 1. Emulator / Sandbox Mode
Use this mode to test and run the visualization without being connected to physical sensors. It simulates an ROV traversing a 3D tunnel structure with wall returns and structures.
* **Run command**: `python mapper_3d_engine.py --emulate`
* **Web Dashboard**: Open `http://localhost:8000` in your web browser.

#### 2. Real Hardware Mode
Executes on real telemetry data. It connects to active serial or UDP devices.
* **Run command**: `python mapper_3d_engine.py --connection <PORT_OR_IP>` (defaults to `192.168.2.2:9092`).
* **Startup Automation**: Run `.\run_mapping_system.ps1` in PowerShell and press `Enter` twice to immediately launch hardware mode using default parameters.
* **Active Retry Loop**: The engine attempts to initialize connection to the Ping360 every 3 seconds if offline, displaying actual status lights on the dashboard.
* **IMU Telemetry**: Listens on UDP Port `14550`. If UDP is unresponsive, it queries the BlueOS REST API bulk messages endpoint (`http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages`) as auto-fallback.
* **DVL Telemetry**: Connects to the Water Linked DVL JSON TCP stream on Port `16171` (IP: `192.168.2.3`).

---

## ⚙️ Configuration Parameter Guide

The WebSocket connection allows the web front-end to tune settings live in the active python mapping session. All settings can be typed directly or slid:

* **IMU Yaw Offset (Default: `270°`)**: Angular correction offset for IMU mounting orientation. Adjustable 0°–360°.
* **Sonar Offset (Default: `0°`)**: Transducer angular offset for Ping360 orientation calibration. Adjustable 0°–360°.
* **Min Range (Default: `0.25 m`)**: Minimum distance cutoff (0.05m–2.0m) to eliminate ROV chassis reflections and transducer main-bang noise while allowing close-up mapping.
* **Intensity Threshold (Default: `110`)**: Return intensity threshold (0–255) below which acoustic noise is filtered out.
* **Speed (Default: `0.20 m/s`)**: Constant forward speed used for dead reckoning if the DVL is offline (only visible/adjustable in Emulator Mode).
* **Scan Step (Default: `1g Fine`)**: Transducer angular step size per ping (1g to 20g).
* **Set Default / Reset**: Click **Set Default** to save current settings into `localStorage` and `settings.json` so they persist across sessions. Click **Reset** to return to factory defaults.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

Copyright © 2026 Coratia Technologies. All rights reserved.
