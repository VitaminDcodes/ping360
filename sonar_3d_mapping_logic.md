# Ping360 3D Sonar Mapping Engine: Mathematical & Logical Documentation

This document explains the mathematical foundations, coordinate transformations, and sensor fusion logic implemented in the **Ping360 3D Mapping Engine**.

---

## 1. System Architecture

The engine fuses three distinct sensor inputs to construct a coherent 3D point cloud:
1. **Blue Robotics Ping360 Sonar**: Returns polar range sweeps (angle $\theta$, distance $d$) in a vertical scan plane.
2. **Mavlink IMU Telemetry**: Provides absolute attitude (Roll $\phi$, Pitch $\theta_p$, Yaw $\psi$) of the ROV.
3. **Water Linked DVL**: Provides local 3D velocities ($v_x, v_y, v_z$) relative to the ROV.

These streams are processed in real-time, aligned to a common global map frame, and streamed to a web dashboard via WebSockets.

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

---

## 2. Coordinate Frames & Conventions

To map local sonar pings into global 3D space, we define three coordinate systems:

### A. Local Sonar Frame ($S$)
The sonar is mounted horizontally at the center-back of the robot. 
* As the transducer rotates, the acoustic beam sweeps a **vertical circle (elevation plane)** perpendicular to the robot's forward movement.
* An acoustic ping at angle $\theta$ (in radians) with distance $d$ (in meters) yields the local Cartesian coordinate:
  $$P_{\text{sonar}} = \begin{bmatrix} x_s \\ y_s \\ z_s \end{bmatrix} = \begin{bmatrix} 0 \\ d \sin\theta \\ d \cos\theta \end{bmatrix}$$
  * $x_s$: Distance along the robot's forward direction (0 within the scan plane).
  * $y_s$: Lateral distance (left/right).
  * $z_s$: Vertical distance (up/down).

### B. Robot Body Frame ($B$)
The robot body frame is centered at the ROV's center of mass.
* **X axis**: Forward (positive forward).
* **Y axis**: Lateral (positive starboard/right).
* **Z axis**: Vertical (positive upwards).
* Since the sonar is mounted at the center-back, we apply a negative longitudinal offset ($x_{\text{offset}} = -0.5\text{ m}$):
  $$P_{\text{robot}} = \begin{bmatrix} x_b \\ y_b \\ z_b \end{bmatrix} = P_{\text{sonar}} + \begin{bmatrix} x_{\text{offset}} \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} x_{\text{offset}} \\ d \sin\theta \\ d \cos\theta \end{bmatrix}$$

### C. Global Map Frame ($G$)
The global map frame is fixed to the starting position of the robot.
* **X axis**: East/Forward path (positive forward).
* **Y axis**: North/Lateral offset (positive left).
* **Z axis**: Elevation/Depth (positive upwards).

---

## 3. Sensor Fusion & Dead Reckoning

To compute the robot's current position and orientation in the Global Frame, we perform real-time dead reckoning.

### A. Attitude Rotation Matrix ($R$)
Using the IMU's roll ($\phi$), pitch ($\theta_p$), and yaw ($\psi$) values, we construct standard Tait-Bryan rotation matrices (using the $Z$-$Y$-$X$ intrinsic convention):

$$R_x(\phi) = \begin{bmatrix} 
1 & 0 & 0 \\ 
0 & \cos\phi & -\sin\phi \\ 
0 & \sin\phi & \cos\phi 
\end{bmatrix}$$

$$R_y(\theta_p) = \begin{bmatrix} 
\cos\theta_p & 0 & \sin\theta_p \\ 
0 & 1 & 0 \\ 
-\sin\theta_p & 0 & \cos\theta_p 
\end{bmatrix}$$

$$R_z(\psi) = \begin{bmatrix} 
\cos\psi & -\sin\psi & 0 \\ 
\sin\psi & \cos\psi & 0 \\ 
0 & 0 & 1 
\end{bmatrix}$$

The combined rotation matrix $R$ transforming from the Robot Frame ($B$) to the Global Frame ($G$) is:
$$R = R_z(\psi) \cdot R_y(\theta_p) \cdot R_x(\phi)$$

### B. Velocity Integration
The DVL streams local velocities $V_{\text{local}} = [v_x, v_y, v_z]^T$ in the Robot Body Frame.
We rotate these velocities into the Global Frame:
$$V_{\text{global}} = R \cdot V_{\text{local}}$$

We then integrate $V_{\text{global}}$ over time step $dt$ to update the global position $T = [p_x, p_y, p_z]^T$:
$$T_{t} = T_{t-dt} + V_{\text{global}} \cdot dt$$

---

## 4. 3D Point Projection

At each time step, for every bin index $i$ in a sonar line returning an intensity value above the threshold:
1. We compute distance: $d = i \times \text{distance\_per\_bin}$.
2. We map it to the Robot Body Frame ($P_{\text{robot}}$).
3. We project it into the Global Map Frame ($P_{\text{global}}$) using the robot's current rotation matrix $R_t$ and translation vector $T_t$:
   $$P_{\text{global}} = R_t \cdot P_{\text{robot}} + T_t$$
4. These projected coordinates are transmitted to the WebGL client for rendering in Three.js.

---

## 5. WebGL Coordinate Mapping (Three.js)

Since Three.js uses a $Y$-up coordinate convention by default:
* Three.js **X** $\rightarrow$ Global **X** (Forward travel)
* Three.js **Y** $\rightarrow$ Global **Z** (Vertical height)
* Three.js **Z** $\rightarrow$ Global **Y** (Lateral shift)

The points are written into a high-performance `Float32Array` buffer attribute and rendered using `THREE.Points` with frustum culling disabled (`frustumCulled = false`) to handle dynamic updates seamlessly.
