# Ping360 3D Sonar Mapping Engine

A real-time 3D underwater scanning and reconstruction system for the **Blue Robotics Ping360 scanning sonar**. 

This system performs live dead-reckoning and coordinate projection by fusing range sweeps from the sonar transducer, attitude telemetry from the ROV's IMU (via Mavlink/ArduSub), and velocity telemetry from a DVL (Water Linked DVL). The output is projected into global 3D coordinates and streamed to a high-performance interactive WebGL dashboard for visual reconstruction.

---

## 🚀 Key Features

*   **Real-Time 3D Reconstruction**: Visualizes sonar sweeps dynamically as a 3D point cloud.
*   **Multi-Sensor Fusion**:
    *   **Blue Robotics Ping360**: Polar acoustic sweeps (angle and intensity/distance bins).
    *   **Mavlink Attitude Telemetry**: Real-time Roll, Pitch, and Yaw to orient the scans.
    *   **Water Linked DVL Telemetry**: Velocity vectors ($v_x, v_y, v_z$) integrated for precise position tracking.
*   **Interactive Three.js Dashboard**:
    *   Sleek light-themed dashboard with complete orbital controls.
    *   Adjustable intensity thresholding, forward speed settings, and sensor offset controls.
    *   Visual representation of the ROV's historical trajectory.
*   **Dual Execution Modes**: Includes an emulator sandbox with simulated tunnel structures to test the software package without hardware.

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

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/VitaminDcodes/ping360.git
    cd ping360
    ```

2.  **Environment Configuration**:
    *   **Windows (PowerShell)**: Run the automated environment setup script. This will create a Python virtual environment and install the required dependencies:
        ```powershell
        .\setup_env.ps1
        ```
    *   **Manual Setup (Cross-platform)**:
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

*   **On Windows**: Double-click `run_mapping_system.bat` or run:
    ```powershell
    .\run_mapping_system.ps1
    ```
*   **Manual launch (Python)**:
    ```bash
    python mapper_3d_engine.py [args]
    ```

### Execution Modes

#### 1. Emulator / Sandbox Mode
Use this mode to test and run the visualization without being connected to physical sensors. It simulates an ROV traversing a 3D tunnel structure with wall returns and structures.
*   **Run command**: `python mapper_3d_engine.py --emulate`
*   **Web Dashboard**: Open `http://localhost:8000` in your web browser.

#### 2. Real Hardware Mode
Executes on real telemetry data. It listens for active serial or UDP devices.
*   **Run command**: `python mapper_3d_engine.py --connection <PORT>` (e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux).
*   **IMU Input**: Listens for Mavlink telemetry broadcast on UDP Port `14550`.
*   **DVL Input**: Connects to the Water Linked DVL JSON stream on TCP Port `16171` (Default IP: `192.168.2.3`).

---

## ⚙️ Configuration Parameter Guide

The WebSocket connection allows the web front-end to tune settings in the active python mapping session. These can be adjusted live in the UI control panel:
*   **Intensity Threshold (Default: `110`)**: Sonar return bin value threshold (0–255) below which noise is filtered.
*   **Speed (Default: `0.20 m/s`)**: Constant forward speed used for dead reckoning if the DVL is offline.
*   **Offset X (Default: `-0.5 m`)**: Mounting distance from the ROV's center of mass along the forward axis.

---

## 📄 License
This project is open-source and available under the MIT License.
