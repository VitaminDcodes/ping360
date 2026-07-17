import argparse
import sys
import time
import math
import asyncio
import threading
import json
import socket
import http.server
import socketserver
import numpy as np
import websockets

# Try importing the Blue Robotics Ping library
try:
    from brping import Ping360
    PING_SDK_AVAILABLE = True
except ImportError:
    PING_SDK_AVAILABLE = False

# Try importing pymavlink for IMU
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

# Global Settings (can be adjusted live via WebSocket controls)
global_settings = {
    "intensity_threshold": 110,
    "speed": 0.20,             # robot forward speed in m/s (when DVL is offline)
    "offset_x": -0.5,          # sensor mounting offset on robot X axis in meters
    "emulate": False,
    "test_imu_fail": False,    # Force IMU offline in software for testing
    "test_dvl_fail": False,    # Force DVL offline in software for testing
    "test_inject_drift": False # Inject systematic position drift for testing
}

# Global State
global_state = {
    "trajectory": [],          # List of dicts: {"x": x, "y": y, "z": z}
    "connected_clients": set(),
    "is_running": True,
    "imu_connected": False,
    "dvl_connected": False,
    "sonar_connected": False,
    "imu_data": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "dvl_data": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "alt": 0.0, "last_update": 0.0},
    "robot_x": 0.0,
    "robot_y": 0.0,
    "robot_z": 0.0,
    "robot_roll": 0.0,
    "robot_pitch": 0.0,
    "robot_yaw": 0.0
}

class Ping360Emulator:
    """Emulates a Ping360 device scanning a circular 3D tunnel environment."""
    def __init__(self, num_samples=250, sample_period=1200):
        self.num_samples = num_samples
        self.sample_period = sample_period  # 25 ns increments
        self.distance_per_bin = 1500.0 * (self.sample_period * 25e-9) / 2.0
        self.range = self.num_samples * self.distance_per_bin
        print(f"[Emulator] Range: {self.range:.2f}m (bin: {self.distance_per_bin*1000:.2f}mm)")

    def initialize(self):
        return True

    def set_gain_setting(self, gain):
        pass

    def set_transmit_frequency(self, freq):
        pass

    def set_number_of_samples(self, num):
        self.num_samples = num
        self.distance_per_bin = 1500.0 * (self.sample_period * 25e-9) / 2.0
        self.range = self.num_samples * self.distance_per_bin

    def set_sample_period(self, period):
        self.sample_period = period
        self.distance_per_bin = 1500.0 * (self.sample_period * 25e-9) / 2.0
        self.range = self.num_samples * self.distance_per_bin

    def transmitAngle(self, angle_gradian, robot_x=0.0):
        """Simulates a ping at a specific angle, returning a mock response."""
        angle_rad = angle_gradian * (2.0 * math.pi / 400.0)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # Circular tunnel of radius 2.2 meters
        wall_distance = 2.2
        
        # Narrowing column support structures every 4.0m along X
        x_mod = robot_x % 4.0
        if x_mod < 0.3:
            wall_distance *= 0.85
            
        # Add a pipe in the top-right corner (y = 1.8, z = 1.5, radius = 0.25)
        pipe_y, pipe_z, pipe_r = 1.8, 1.5, 0.25
        a_quad = 1.0
        b_quad = -2.0 * (pipe_y * sin_a + pipe_z * cos_a)
        c_quad = pipe_y**2 + pipe_z**2 - pipe_r**2
        discriminant = b_quad**2 - 4 * a_quad * c_quad
        if discriminant >= 0:
            d_p1 = (-b_quad - math.sqrt(discriminant)) / (2.0 * a_quad)
            d_p2 = (-b_quad + math.sqrt(discriminant)) / (2.0 * a_quad)
            for dp in (d_p1, d_p2):
                if 0 < dp < wall_distance:
                    wall_distance = dp
                    
        # Add small surface noise
        wall_distance += np.random.normal(0, 0.015)
        wall_distance = min(wall_distance, self.range - 0.1)
        
        wall_bin = int(wall_distance / self.distance_per_bin)
        
        # Background noise
        intensities = np.random.randint(2, 20, size=self.num_samples, dtype=np.uint8)
        
        # Generate peak
        if 0 <= wall_bin < self.num_samples:
            for idx in range(max(0, wall_bin - 5), min(self.num_samples, wall_bin + 6)):
                diff = idx - wall_bin
                peak_val = int(230 * math.exp(-(diff**2) / 3.0))
                intensities[idx] = min(255, max(intensities[idx], peak_val))
                
        class MockResponse:
            def __init__(self, angle, data, num_samples, sample_period):
                self.angle = angle
                self.data = bytearray(data)
                self.number_of_samples = num_samples
                self.sample_period = sample_period
                
        return MockResponse(angle_gradian, intensities, self.num_samples, self.sample_period)


# --- BACKGROUND CLIENT THREADS ---

def mavlink_imu_thread():
    """Reads attitude telemetry from Mavlink UDP stream at 192.168.2.2:14550."""
    global global_state, global_settings
    if not MAVLINK_AVAILABLE:
        print("[Mavlink] Pymavlink library not found. Running in emulated attitude mode.")
        return
        
    print("[Mavlink] Listening for Mavlink telemetry from 192.168.2.2:14550...")
    while global_state["is_running"]:
        if global_settings["test_imu_fail"]:
            global_state["imu_connected"] = False
            time.sleep(0.5)
            continue
            
        try:
            # Connect as udpout (send heartbeats to 192.168.2.2, listen on bound port)
            master = mavutil.mavlink_connection("udpout:192.168.2.2:14550", source_system=255)
            last_msg_time = time.time()
            
            while global_state["is_running"] and not global_settings["test_imu_fail"]:
                # Non-blocking read (timeout 0.5s)
                msg = master.recv_match(type=['ATTITUDE', 'HEARTBEAT'], blocking=True, timeout=0.5)
                if msg:
                    msg_type = msg.get_type()
                    if msg_type == 'ATTITUDE':
                        global_state["imu_connected"] = True
                        global_state["imu_data"]["roll"] = msg.roll
                        global_state["imu_data"]["pitch"] = msg.pitch
                        global_state["imu_data"]["yaw"] = msg.yaw
                        last_msg_time = time.time()
                    elif msg_type == 'HEARTBEAT':
                        # Autopilot connected, keep connection alive
                        master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
                        )
                
                # Check for link timeout (3 seconds)
                if time.time() - last_msg_time > 3.0:
                    global_state["imu_connected"] = False
                    
            master.close()
        except Exception as e:
            global_state["imu_connected"] = False
            time.sleep(2.0)


def dvl_client_thread():
    """Connects to DVL TCP server at 192.168.2.3:16171 (Water Linked JSON API)."""
    global global_state, global_settings
    print("[DVL] Client worker initialized.")
    
    while global_state["is_running"]:
        if global_settings["test_dvl_fail"]:
            global_state["dvl_connected"] = False
            time.sleep(0.5)
            continue
            
        try:
            # Attempt TCP Connection
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect(("192.168.2.3", 16171))
            print("[DVL] Connected to Water Linked TCP API at 192.168.2.3:16171")
            global_state["dvl_connected"] = True
            
            buffer = ""
            while global_state["is_running"] and not global_settings["test_dvl_fail"]:
                data = sock.recv(1024)
                if not data:
                    break
                    
                buffer += data.decode('utf-8')
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line.strip():
                        try:
                            msg = json.loads(line)
                            # JSON: {"vx": ..., "vy": ..., "vz": ..., "valid": true}
                            if msg.get("valid", False):
                                global_state["dvl_connected"] = True
                                global_state["dvl_data"]["vx"] = msg.get("vx", 0.0)
                                global_state["dvl_data"]["vy"] = msg.get("vy", 0.0)
                                global_state["dvl_data"]["vz"] = msg.get("vz", 0.0)
                                global_state["dvl_data"]["alt"] = msg.get("alt", 0.0)
                                global_state["dvl_data"]["last_update"] = time.time()
                        except Exception:
                            pass
                            
            sock.close()
            global_state["dvl_connected"] = False
        except Exception:
            global_state["dvl_connected"] = False
            time.sleep(2.0)


# --- WEB SERVING & TELEMETRY BROADCAST ---

def run_http_server():
    """Serves the static index.html dashboard on Port 8000."""
    PORT = 8000
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # Suppress console logging to keep terminal output clean
            
        def end_headers(self):
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            super().end_headers()
            
    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.TCPServer(("", PORT), QuietHandler) as httpd:
            print(f"[HTTP] Web Dashboard served at http://localhost:{PORT}")
            httpd.serve_forever()
    except Exception as e:
        print(f"[HTTP] Server error: {e}")

async def broadcast(message_str):
    """Sends a message to all connected WebSocket clients."""
    if global_state["connected_clients"]:
        tasks = [asyncio.create_task(client.send(message_str)) for client in global_state["connected_clients"]]
        await asyncio.gather(*tasks, return_exceptions=True)

async def send_trajectory_history(websocket):
    """Sends the entire stored trajectory path to a newly connected client."""
    msg = {
        "type": "trajectory",
        "path": global_state["trajectory"]
    }
    await websocket.send(json.dumps(msg))

async def ws_handler(websocket):
    """Manages individual WebSocket client connections and incoming command requests."""
    global_state["connected_clients"].add(websocket)
    
    try:
        await send_trajectory_history(websocket)
        
        async for message in websocket:
            data = json.loads(message)
            action = data.get("action")
            
            if action == "update_settings":
                global_settings["intensity_threshold"] = data.get("intensity_threshold", 110)
                global_settings["speed"] = data.get("speed", 0.20)
                global_settings["test_imu_fail"] = data.get("test_imu_fail", False)
                global_settings["test_dvl_fail"] = data.get("test_dvl_fail", False)
                global_settings["test_inject_drift"] = data.get("test_inject_drift", False)
                
            elif action == "clear":
                global_state["trajectory"].clear()
                global_state["robot_x"] = 0.0
                global_state["robot_y"] = 0.0
                global_state["robot_z"] = 0.0
                print("[WS] Reset navigation coordinates.")
                
    except Exception:
        pass
    finally:
        global_state["connected_clients"].remove(websocket)


# --- CORE NAVIGATION AND MAPPING PIPELINE ---

def update_dead_reckoning(dt, current_time):
    """Computes the robot's current pose using DVL/IMU fusion or simulated dead reckoning."""
    emulate = global_settings.get("emulate", False)
    
    # 1. Position Step
    if global_state["dvl_connected"] and not global_settings["test_dvl_fail"]:
        # Real DVL data is integrated below after rotation matrix is built
        pass
    else:
        if emulate:
            # Emulator Mode: Crawl forward along X axis
            global_state["robot_x"] += global_settings["speed"] * dt
            # Follow a curved trajectory after 5 meters (curves to left along Y, downward along Z)
            if global_state["robot_x"] < 5.0:
                global_state["robot_y"] = 0.0
                global_state["robot_z"] = 0.0
            else:
                dx = global_state["robot_x"] - 5.0
                global_state["robot_y"] = 0.04 * (dx ** 2)
                global_state["robot_z"] = -0.015 * (dx ** 2)
        else:
            # Real hardware mode: DVL is disconnected/failed. Do not integrate fake motion.
            pass

    # 2. Orientation Step (IMU / Trajectory heading)
    if global_state["imu_connected"] and not global_settings["test_imu_fail"]:
        roll = global_state["imu_data"]["roll"]
        pitch = global_state["imu_data"]["pitch"]
        yaw = global_state["imu_data"]["yaw"]
    else:
        if emulate:
            # Emulated orientation aligned with the trajectory tangent
            roll = 0.04 * math.sin(0.25 * current_time)
            if global_state["robot_x"] < 5.0:
                yaw = 0.0
                pitch = 0.0
            else:
                dx = global_state["robot_x"] - 5.0
                # Tangent angle for y = 0.04 * dx^2 is dy/dx = 0.08 * dx
                yaw = math.atan(0.08 * dx)
                # Tangent angle for z = -0.015 * dx^2 is dz/dx = -0.03 * dx
                pitch = -math.atan(0.03 * dx)
        else:
            # Real hardware mode: IMU is disconnected. Maintain last attitude or default to zero.
            roll = global_state.get("robot_roll", 0.0)
            pitch = global_state.get("robot_pitch", 0.0)
            yaw = global_state.get("robot_yaw", 0.0)

    global_state["robot_roll"] = roll
    global_state["robot_pitch"] = pitch
    global_state["robot_yaw"] = yaw
    
    # Combined Rotation Matrix
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    R = Rz @ Ry @ Rx
    
    # Complete DVL position integration if DVL is connected
    if global_state["dvl_connected"] and not global_settings["test_dvl_fail"]:
        vx_local = global_state["dvl_data"]["vx"]
        vy_local = global_state["dvl_data"]["vy"]
        vz_local = global_state["dvl_data"]["vz"]
        v_local = np.array([vx_local, vy_local, vz_local])
        v_global = R @ v_local
        
        global_state["robot_x"] += v_global[0] * dt
        global_state["robot_y"] += v_global[1] * dt
        global_state["robot_z"] += v_global[2] * dt

    # 3. Test Case: Inject Systematic Position Drift
    if global_settings["test_inject_drift"]:
        drift_y = (0.015 + np.random.normal(0, 0.04)) * dt
        drift_z = (0.010 + np.random.normal(0, 0.03)) * dt
        global_state["robot_y"] += drift_y
        global_state["robot_z"] += drift_z
        
    t_vec = np.array([global_state["robot_x"], global_state["robot_y"], global_state["robot_z"]])
    return R, t_vec



def process_sonar_line(response, R_pose, t_pose, angle_gradian):
    """Processes 2D polar sonar scans into 3D Cartesian coordinates fused with robot pose."""
    intensities = np.frombuffer(response.data, dtype=np.uint8)
    distance_per_bin = 1500.0 * (response.sample_period * 25e-9) / 2.0
    
    # Gradian to radians
    angle_rad = angle_gradian * (2.0 * math.pi / 400.0)
    dir_y = math.sin(angle_rad)
    dir_z = math.cos(angle_rad)
    
    points_to_send = []
    
    threshold = global_settings["intensity_threshold"]
    valid_indices = np.where(intensities >= threshold)[0]
    
    # Subsample indices to keep operations responsive
    if len(valid_indices) > 30:
        valid_indices = valid_indices[::2]
        
    for bin_idx in valid_indices:
        intensity = int(intensities[bin_idx])
        d = bin_idx * distance_per_bin
        
        if d < 0.25:
            continue
            
        # Sonar Y-Z vertical plane scan
        x_local = 0.0
        y_local = d * dir_y
        z_local = d * dir_z
        
        # Apply center-back mounting offset
        x_robot = x_local + global_settings["offset_x"]
        y_robot = y_local
        z_robot = z_local
        
        # Robot Frame -> Global Map Frame
        P_robot = np.array([x_robot, y_robot, z_robot])
        P_global = R_pose @ P_robot + t_pose
        
        points_to_send.append({
            "x": float(P_global[0]),
            "y": float(P_global[1]),
            "z": float(P_global[2]),
            "intensity": intensity,
            "angle": angle_gradian
        })
        
    # Diagnostic logging
    if points_to_send and angle_gradian % 20 == 0:
        print(f"[Engine] Angle {angle_gradian}: Generated {len(points_to_send)} points (Robot X: {t_pose[0]:.2f})", flush=True)
        
    return points_to_send


async def run_sonar_engine():
    """Main query, computation, and WebSocket broadcast loop."""
    print("[System] Connecting to Ping360 sonar...")
    
    device = None
    emulated = False
    
    if global_settings["emulate"] or not PING_SDK_AVAILABLE:
        device = Ping360Emulator(num_samples=250, sample_period=1200)
        emulated = True
        global_state["sonar_connected"] = False
        print("[System] Running in Sonar EMULATED mode.")
    else:
        device = Ping360()
        conn = args.connection
        if "." in conn:
            device.connect_udp(conn, args.udp_port)
        else:
            device.connect_serial(conn, args.baudrate)
            
        if device.initialize():
            device.set_gain_setting(1)
            device.set_number_of_samples(250)
            device.set_sample_period(1200)
            device.set_transmit_frequency(750)
            print("[System] Connected to physical Ping360 Hardware.")
            global_state["sonar_connected"] = True
        else:
            print("[System] Ping360 connection failed! Falling back to Emulator.")
            device = Ping360Emulator(num_samples=250, sample_period=1200)
            emulated = True
            global_state["sonar_connected"] = False
            
    last_loop_time = time.time()
    start_time = time.time()
    angle = 0
    last_trajectory_time = 0
    
    try:
        while global_state["is_running"]:
            current_time = time.time() - start_time
            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now
            
            # Avoid huge dt values on initial startups
            if dt > 0.2:
                dt = 0.05
                
            # 1. Update pose via dead reckoning
            R_pose, t_pose = update_dead_reckoning(dt, current_time)
            
            # Store trajectory nodes periodically
            if current_time - last_trajectory_time > 0.35:
                pos_dict = {"x": float(t_pose[0]), "y": float(t_pose[1]), "z": float(t_pose[2])}
                global_state["trajectory"].append(pos_dict)
                last_trajectory_time = current_time
                
                await broadcast(json.dumps({
                    "type": "trajectory",
                    "path": global_state["trajectory"]
                }))
                
            # 2. Query scan line (transmitAngle is blocking, run in separate thread)
            if emulated:
                response = device.transmitAngle(angle, robot_x=t_pose[0])
            else:
                response = await asyncio.to_thread(device.transmitAngle, angle)
                
            if response:
                points = process_sonar_line(response, R_pose, t_pose, angle)
                if points:
                    await broadcast(json.dumps({
                        "type": "points",
                        "points": points
                    }))
                    
            # 3. Broadcast telemetry packet including connection statuses
            telemetry_data = {
                "type": "telemetry",
                "scan_angle": angle,
                "sonar_connected": global_state["sonar_connected"] or emulated,
                "imu_connected": (global_state["imu_connected"] or emulated) and not global_settings["test_imu_fail"],
                "dvl_connected": (global_state["dvl_connected"] or emulated) and not global_settings["test_dvl_fail"],
                "robot_pose": {
                    "x": float(t_pose[0]),
                    "y": float(t_pose[1]),
                    "z": float(t_pose[2]),
                    "roll": float(global_state["robot_roll"]),
                    "pitch": float(global_state["robot_pitch"]),
                    "yaw": float(global_state["robot_yaw"])
                }
            }
            await broadcast(json.dumps(telemetry_data))
            
            angle = (angle + 1) % 400
            
            await asyncio.sleep(0.01)
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f"[Engine] Exception in main loop: {e}")


async def main(args):
    global_settings["emulate"] = args.emulate
    global_settings["offset_x"] = args.offset_x
    global_settings["intensity_threshold"] = args.intensity_threshold
    global_settings["speed"] = args.speed
    
    # Start HTTP serving
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    
    # Start sensor reader threads
    imu_worker = threading.Thread(target=mavlink_imu_thread, daemon=True)
    dvl_worker = threading.Thread(target=dvl_client_thread, daemon=True)
    
    imu_worker.start()
    dvl_worker.start()
    
    # Start WebSocket Server
    async with websockets.serve(ws_handler, "localhost", 8001) as server:
        print("[WS] WebSocket Server listening on ws://localhost:8001")
        
        # Launch mapping task
        engine_task = asyncio.create_task(run_sonar_engine())
        
        try:
            await asyncio.Future()  # run forever
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("\n[System] Stopping servers...")
        finally:
            global_state["is_running"] = False
            engine_task.cancel()
            await engine_task

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ping360 Real-Time 3D Mapping & Sensor Fusion Engine")
    parser.add_argument("--connection", type=str, default="COM3", help="Serial port (e.g. COM3) or IP address of Ping360")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baudrate for serial connection (default 115200)")
    parser.add_argument("--udp-port", type=int, default=9090, help="UDP port (default 9090)")
    parser.add_argument("--emulate", action="store_true", help="Force emulation mode")
    parser.add_argument("--intensity-threshold", type=int, default=110, help="Intensity threshold (0-255) for mapping points")
    parser.add_argument("--offset-x", type=float, default=-0.5, help="Sonar mounting offset along robot's X axis in meters")
    parser.add_argument("--speed", type=float, default=0.20, help="Robot forward speed in m/s")
    
    args = parser.parse_args()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nShutdown complete.")
