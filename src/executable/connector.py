import sys
import cv2
import numpy as np
import socket
import json
import threading
import struct
import select
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QSlider,
                             QLineEdit, QGroupBox, QGridLayout, QTabWidget)
from PyQt5.QtCore import Qt, pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap
import pyrealsense2 as rs
import mediapipe as mp


# ============================================================================
# CameraThread class - Handles RealSense camera and hand tracking
# ============================================================================

class CameraThread(QThread):
    """Handles RealSense camera and hand tracking"""
    frame_ready = pyqtSignal(np.ndarray)
    hand_data = pyqtSignal(np.ndarray, float)  # position, bend_angle

    def __init__(self):
        super().__init__()
        self.running = False

    def run(self):
        # Initialize MediaPipe
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)
        mp_draw = mp.solutions.drawing_utils

        # Initialize RealSense
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        align = rs.align(rs.stream.color)
        self.running = True

        while self.running:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Process hand
            results = hands.process(rgb_image)

            if results.multi_hand_landmarks:
                hand = results.multi_hand_landmarks[0]
                mp_draw.draw_landmarks(color_image, hand, mp_hands.HAND_CONNECTIONS)

                # Get hand center position
                h, w = color_image.shape[:2]
                cx = int(sum([lm.x for lm in hand.landmark]) / 21 * w)
                cy = int(sum([lm.y for lm in hand.landmark]) / 21 * h)

                # Clamp coordinates to valid frame bounds (prevent out of range errors)
                cx = max(0, min(cx, w - 1))
                cy = max(0, min(cy, h - 1))

                try:
                    # Get 3D position
                    depth = depth_frame.get_distance(cx, cy)
                    
                    # Skip if depth is invalid (0 or too far)
                    if depth == 0 or depth > 5.0:
                        # Still draw the hand but don't emit data
                        cv2.circle(color_image, (cx, cy), 10, (0, 0, 255), -1)  # Red for invalid depth
                        continue
                    
                    intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(intrin, [cx, cy], depth)
                    position = np.array([[point_3d[0]], [point_3d[1]], [point_3d[2]]])

                    # Calculate finger bend (index finger)
                    mcp, pip, dip = hand.landmark[5], hand.landmark[6], hand.landmark[7]
                    v1 = np.array([pip.x - mcp.x, pip.y - mcp.y, pip.z - mcp.z])
                    v2 = np.array([dip.x - pip.x, dip.y - pip.y, dip.z - pip.z])
                    angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) /
                                       (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1)))

                    # Draw center point in green for valid tracking
                    cv2.circle(color_image, (cx, cy), 10, (0, 255, 0), -1)

                    self.hand_data.emit(position, angle)
                    
                except (RuntimeError, Exception) as e:
                    # Handle any RealSense errors gracefully
                    print(f"[Camera] Error getting hand position: {e}")
                    # Draw center point in orange for error state
                    cv2.circle(color_image, (cx, cy), 10, (0, 165, 255), -1)
                    continue

            self.frame_ready.emit(color_image)

        hands.close()
        pipeline.stop()

    def stop(self):
        self.running = False


# ============================================================================
# PiWebcamThread class - Receives webcam stream from Raspberry Pi
# ============================================================================

class PiWebcamThread(QThread):
    """
    Receives webcam stream from Raspberry Pi with low-latency optimizations:
    - JPEG decoding (matches server's JPEG encoding)
    - Frame dropping to always display newest frame
    - Non-blocking socket with select()
    - TCP_NODELAY and reduced buffer sizes
    """
    frame_ready = pyqtSignal(np.ndarray)
    connection_status = pyqtSignal(str)

    def __init__(self, pi_ip):
        super().__init__()
        self.pi_ip = pi_ip
        self.running = False
        self.socket = None

    def run(self):
        try:
            self.connection_status.emit("Connecting to Pi webcam...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Optimize socket for low latency
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.socket.settimeout(5)
            self.socket.connect((self.pi_ip, 8001))
            self.socket.settimeout(0.1)  # Short timeout for non-blocking reads
            self.connection_status.emit("Pi webcam connected")
            self.running = True

            data_buffer = b""
            payload_size = struct.calcsize("Q")

            while self.running:
                try:
                    # Use select to check if data is available (non-blocking)
                    ready = select.select([self.socket], [], [], 0.01)
                    if not ready[0]:
                        continue

                    # Receive all available data (drain the buffer to get latest frame)
                    while True:
                        try:
                            chunk = self.socket.recv(65536)
                            if not chunk:
                                self.connection_status.emit("Pi webcam disconnected")
                                self.running = False
                                break
                            data_buffer += chunk
                        except socket.timeout:
                            break  # No more data available right now
                        except BlockingIOError:
                            break

                    if not self.running:
                        break

                    # Process all complete frames, keep only the last one
                    last_frame = None
                    while len(data_buffer) >= payload_size:
                        # Extract message size
                        msg_size = struct.unpack("Q", data_buffer[:payload_size])[0]

                        # Check if we have complete frame
                        if len(data_buffer) < payload_size + msg_size:
                            break  # Wait for more data

                        # Extract frame data
                        frame_data = data_buffer[payload_size:payload_size + msg_size]
                        data_buffer = data_buffer[payload_size + msg_size:]

                        # Decode JPEG frame
                        frame_array = np.frombuffer(frame_data, dtype=np.uint8)
                        frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                        if frame is not None:
                            last_frame = frame

                    # Only emit the most recent frame (drop older ones)
                    if last_frame is not None:
                        self.frame_ready.emit(last_frame)

                except Exception as e:
                    if self.running:
                        self.connection_status.emit(f"Pi webcam error: {e}")
                    break

        except Exception as e:
            self.connection_status.emit(f"Pi webcam error: {e}")
            print(f"Pi webcam error: {e}")
        finally:
            if self.socket:
                self.socket.close()
            self.connection_status.emit("Pi webcam disconnected")

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass


# ============================================================================
# Main Controller Class
# ============================================================================

class RobotArmController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller")
        self.setGeometry(100, 100, 950, 850)

        self.pi_socket = None
        self.camera_thread = None
        self.pi_webcam_thread = None
        self.tracking_mode = False
        
        # Default servo positions (middle of ranges)
        self.servo_positions = [45, 105, 105, 125]
        self.speed = 50
        
        # Rate limiting and change detection
        self.last_servo_send_time = 0
        self.servo_send_interval = 0.1  # Max 20 Hz (50ms between commands)
        self.last_sent_positions = [45, 105, 105, 125]
        self.servo_change_threshold = 5  # Send if any servo changed by 1+ degree

        self.init_ui()

    def init_ui(self):
        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)

        # Connection
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("Pi IP:"))
        self.ip_input = QLineEdit("100.71.223.50")
        conn_layout.addWidget(self.ip_input)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        self.status_label = QLabel("Disconnected")
        conn_layout.addWidget(self.status_label)
        layout.addLayout(conn_layout)

        # Tab widget for camera feeds
        self.tab_widget = QTabWidget()

        # RealSense tab
        realsense_widget = QWidget()
        realsense_layout = QVBoxLayout(realsense_widget)
        self.camera_label = QLabel("Click 'Start RealSense' to begin")
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid black;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        realsense_layout.addWidget(self.camera_label)

        # Hand info
        self.hand_info = QLabel("Hand Position: N/A | Finger Bend: N/A")
        realsense_layout.addWidget(self.hand_info)

        self.tab_widget.addTab(realsense_widget, "RealSense Camera")

        # Pi Webcam tab
        pi_webcam_widget = QWidget()
        pi_webcam_layout = QVBoxLayout(pi_webcam_widget)
        self.pi_webcam_label = QLabel("Connect to Pi and click 'Start Pi Webcam'")
        self.pi_webcam_label.setMinimumSize(640, 480)
        self.pi_webcam_label.setStyleSheet("border: 2px solid blue;")
        self.pi_webcam_label.setAlignment(Qt.AlignCenter)
        pi_webcam_layout.addWidget(self.pi_webcam_label)

        # Webcam status
        self.webcam_status_label = QLabel("Status: Not connected")
        pi_webcam_layout.addWidget(self.webcam_status_label)

        self.tab_widget.addTab(pi_webcam_widget, "Pi Webcam")

        layout.addWidget(self.tab_widget)

        # Controls
        ctrl_layout = QHBoxLayout()
        self.camera_btn = QPushButton("Start RealSense")
        self.camera_btn.clicked.connect(self.toggle_camera)
        ctrl_layout.addWidget(self.camera_btn)

        self.pi_webcam_btn = QPushButton("Start Pi Webcam")
        self.pi_webcam_btn.clicked.connect(self.toggle_pi_webcam)
        self.pi_webcam_btn.setEnabled(False)
        ctrl_layout.addWidget(self.pi_webcam_btn)

        self.track_btn = QPushButton("Start Tracking Mode")
        self.track_btn.clicked.connect(self.toggle_tracking)
        self.track_btn.setEnabled(False)
        ctrl_layout.addWidget(self.track_btn)
        layout.addLayout(ctrl_layout)

        # Speed control
        speed_group = QGroupBox("Movement Speed")
        speed_layout = QHBoxLayout(speed_group)
        speed_layout.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.speed_changed)
        speed_layout.addWidget(self.speed_slider)
        self.speed_label = QLabel("50")
        speed_layout.addWidget(self.speed_label)
        layout.addWidget(speed_group)

        # Servos with button controls - CENTERED
        servo_group = QGroupBox("Manual Servo Control")
        servo_main_layout = QHBoxLayout(servo_group)
        
        # Add stretches on both sides to center the content
        servo_main_layout.addStretch()
        
        # Create the actual servo control grid
        servo_layout = QGridLayout()
        
        self.angle_inputs = []  # Store QLineEdit widgets for angle display/input
        
        # Servo ranges and defaults
        servo_names = ["Base (0-90°)", "Arm1 (70-140°)", "Arm2 (80-130°)", "Gripper (90-160°)"]
        servo_ranges = [(0, 90), (70, 140), (80, 130), (90, 160)]
        servo_defaults = [45, 105, 105, 125]  # Middle positions for all servos

        for i in range(4):
            # Servo label
            label = QLabel(servo_names[i])
            label.setMinimumWidth(120)
            servo_layout.addWidget(label, i, 0)
            
            # -10 button
            btn_minus_10 = QPushButton("-10")
            btn_minus_10.setFixedWidth(60)
            btn_minus_10.clicked.connect(lambda checked, idx=i: self.adjust_angle(idx, -10))
            servo_layout.addWidget(btn_minus_10, i, 1)
            
            # -1 button
            btn_minus_1 = QPushButton("-1")
            btn_minus_1.setFixedWidth(50)
            btn_minus_1.clicked.connect(lambda checked, idx=i: self.adjust_angle(idx, -1))
            servo_layout.addWidget(btn_minus_1, i, 2)
            
            # Angle input/display textbox
            angle_input = QLineEdit(str(servo_defaults[i]))
            angle_input.setFixedWidth(70)
            angle_input.setAlignment(Qt.AlignCenter)
            angle_input.setProperty("servo_index", i)
            angle_input.returnPressed.connect(self.on_manual_entry)
            self.angle_inputs.append(angle_input)
            servo_layout.addWidget(angle_input, i, 3)
            
            # +1 button
            btn_plus_1 = QPushButton("+1")
            btn_plus_1.setFixedWidth(50)
            btn_plus_1.clicked.connect(lambda checked, idx=i: self.adjust_angle(idx, 1))
            servo_layout.addWidget(btn_plus_1, i, 4)
            
            # +10 button
            btn_plus_10 = QPushButton("+10")
            btn_plus_10.setFixedWidth(60)
            btn_plus_10.clicked.connect(lambda checked, idx=i: self.adjust_angle(idx, 10))
            servo_layout.addWidget(btn_plus_10, i, 5)

        servo_main_layout.addLayout(servo_layout)
        servo_main_layout.addStretch()
        
        layout.addWidget(servo_group)

    def toggle_connection(self):
        if self.pi_socket:
            self.pi_socket.close()
            self.pi_socket = None
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.track_btn.setEnabled(False)
            self.pi_webcam_btn.setEnabled(False)
        else:
            try:
                self.pi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.pi_socket.connect((self.ip_input.text(), 8000))
                self.connect_btn.setText("Disconnect")
                self.status_label.setText("Connected")
                self.track_btn.setEnabled(True)
                self.pi_webcam_btn.setEnabled(True)
            except Exception as e:
                self.status_label.setText(f"Error: {e}")

    def toggle_camera(self):
        if self.camera_thread and self.camera_thread.isRunning():
            self.camera_thread.stop()
            self.camera_thread.wait()
            self.camera_btn.setText("Start RealSense")
        else:
            self.camera_thread = CameraThread()
            self.camera_thread.frame_ready.connect(self.update_frame)
            self.camera_thread.hand_data.connect(self.update_hand_data)
            self.camera_thread.start()
            self.camera_btn.setText("Stop RealSense")

    def toggle_pi_webcam(self):
        if self.pi_webcam_thread and self.pi_webcam_thread.isRunning():
            self.pi_webcam_thread.stop()
            self.pi_webcam_thread.wait()
            self.pi_webcam_btn.setText("Start Pi Webcam")
            self.webcam_status_label.setText("Status: Stopped")
        else:
            self.pi_webcam_thread = PiWebcamThread(self.ip_input.text())
            self.pi_webcam_thread.frame_ready.connect(self.update_pi_webcam_frame)
            self.pi_webcam_thread.connection_status.connect(self.update_webcam_status)
            self.pi_webcam_thread.start()
            self.pi_webcam_btn.setText("Stop Pi Webcam")

    def toggle_tracking(self):
        self.tracking_mode = not self.tracking_mode
        self.track_btn.setText("Stop Tracking" if self.tracking_mode else "Start Tracking Mode")
        
        # Disable/enable angle input boxes in tracking mode
        for angle_input in self.angle_inputs:
            angle_input.setEnabled(not self.tracking_mode)

    def speed_changed(self, value):
        self.speed = value
        self.speed_label.setText(str(value))

    def adjust_angle(self, idx, delta):
        """Adjust servo angle by delta amount"""
        if self.tracking_mode:
            return  # Don't allow manual adjustments in tracking mode
        
        servo_ranges = [(0, 90), (70, 140), (80, 130), (90, 160)]
        min_angle, max_angle = servo_ranges[idx]
        
        current = self.servo_positions[idx]
        new_angle = max(min_angle, min(max_angle, current + delta))
        self.servo_positions[idx] = new_angle
        self.angle_inputs[idx].setText(str(new_angle))
        self.send_servos()

    def manual_angle_entry(self, idx):
        """Handle manual angle entry from textbox"""
        if self.tracking_mode:
            self.angle_inputs[idx].setText(str(self.servo_positions[idx]))
            return
        
        servo_ranges = [(0, 90), (70, 140), (80, 130), (90, 160)]
        min_angle, max_angle = servo_ranges[idx]
        
        try:
            new_angle = int(self.angle_inputs[idx].text())
            new_angle = max(min_angle, min(max_angle, new_angle))
            self.servo_positions[idx] = new_angle
            self.angle_inputs[idx].setText(str(new_angle))
            self.send_servos()
        except ValueError:
            self.angle_inputs[idx].setText(str(self.servo_positions[idx]))

    def on_manual_entry(self):
        """Handle manual angle entry - gets the index from the sender"""
        sender = self.sender()
        idx = sender.property("servo_index")
        self.manual_angle_entry(idx)

    def update_frame(self, frame):
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qt_img.rgbSwapped()).scaled(
            640, 480, Qt.KeepAspectRatio))

    def update_pi_webcam_frame(self, frame):
        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        self.pi_webcam_label.setPixmap(QPixmap.fromImage(qt_img.rgbSwapped()).scaled(
            640, 480, Qt.KeepAspectRatio))

    def update_webcam_status(self, status):
        self.webcam_status_label.setText(f"Status: {status}")

    # ========================================================================
    # DIRECT CAMERA-TO-SERVO MAPPING (No IK)
    # ========================================================================

    def update_hand_data(self, position, bend_angle):
        """
        Process hand tracking data and directly map to servo positions.
        NO inverse kinematics - direct mapping from camera space to servo space.

        Camera coordinate system:
            X: Left/right (-X = left, +X = right when facing camera)
            Y: Up/down   (-Y = up, +Y = down in image coordinates)
            Z: Depth     (distance from camera, always positive)

        Servo mapping:
            X (left/right) → Base  servo (0-90°)
            Z (depth/in-out) → Arm1 servo (70-140°)   ← SWAPPED: was Y
            Y (up/down)      → Arm2 servo (80-130°)   ← SWAPPED: was Z
            Finger bend      → Gripper servo (90-160°)

        Rate limiting:
            - Maximum 20 Hz update rate (50ms between commands)
            - Only sends if any servo changed by threshold degrees
        """
        # Safety check: Validate position data
        if position is None or position.size == 0:
            self.hand_info.setText("Hand Position: Lost | Finger: N/A")
            return

        try:
            # Extract hand position coordinates (in meters)
            x_cam, y_cam, z_cam = position[0][0], position[1][0], position[2][0]

            # Check for invalid depth values
            if not np.isfinite(x_cam) or not np.isfinite(y_cam) or not np.isfinite(z_cam) or z_cam == 0:
                self.hand_info.setText("Hand Position: Invalid depth | Finger: N/A")
                return

            # Define camera workspace bounds (detection range)
            x_cam_min, x_cam_max = -0.5, 0.7   # Left/right range
            y_cam_min, y_cam_max = -0.2, 0.35  # Up/down range
            z_cam_min, z_cam_max = 0.3, 0.75   # Depth range

            # Clamp camera coordinates to detection bounds
            x_cam = np.clip(x_cam, x_cam_min, x_cam_max)
            y_cam = np.clip(y_cam, y_cam_min, y_cam_max)
            z_cam = np.clip(z_cam, z_cam_min, z_cam_max)

            # Update display with camera position
            self.hand_info.setText(
                f"Cam: X={x_cam:.2f}m Y={y_cam:.2f}m Z={z_cam:.2f}m | Finger: {bend_angle:.0f}°"
            )

        except (IndexError, TypeError):
            self.hand_info.setText("Hand Position: Error reading data | Finger: N/A")
            return

        # Only control robot when tracking mode is active
        if self.tracking_mode:
            # DIRECT MAPPING: Camera coordinates → Servo angles

            # Base servo (0-90°): Map X position (left/right)
            # x_cam: -0.5 (left) to +0.7 (right) → servo: 0° to 90°
            base_servo = np.interp(x_cam, [x_cam_min, x_cam_max], [0, 90])

            # Arm1 servo (70-140°): Map Z position (depth / hand in-out)
            # z_cam: 0.3 (close) to 0.75 (far) → servo: 140° to 70°
            # Hand closer to camera → higher angle
            arm1_servo = np.interp(z_cam, [z_cam_min, z_cam_max], [70, 140])

            # Arm2 servo (80-130°): Map Y position (up/down / hand height)
            # y_cam: -0.2 (up) to +0.35 (down) → servo: 80° to 130°
            # Hand higher (more negative Y) → lower angle
            arm2_servo = np.interp(y_cam, [y_cam_min, y_cam_max], [80, 130])

            # Gripper servo (90-160°): Map finger bend angle
            # bend_angle: 0° (straight) to 180° (bent) → servo: 160° (open) to 90° (closed)
            gripper_servo = np.interp(bend_angle, [0, 180], [160, 90])

            # Clamp to physical servo limits and convert to integers
            servo_angles = [
                int(np.clip(base_servo,    0,  90)),
                int(np.clip(arm1_servo,   70, 140)),
                int(np.clip(arm2_servo,   80, 130)),
                int(np.clip(gripper_servo, 90, 160))
            ]

            # Update internal servo positions
            self.servo_positions = servo_angles

            # Update UI to show current servo positions
            for i in range(4):
                self.angle_inputs[i].setText(str(self.servo_positions[i]))

            # Rate limiting + change detection
            import time
            current_time = time.time()

            time_passed = current_time - self.last_servo_send_time >= self.servo_send_interval
            position_changed = any(
                abs(self.servo_positions[i] - self.last_sent_positions[i]) >= self.servo_change_threshold
                for i in range(4)
            )

            if time_passed and position_changed:
                self.send_servos()
                self.last_servo_send_time = current_time
                self.last_sent_positions = self.servo_positions.copy()

                print(f"[Tracking] Cam: X={x_cam:.2f} Y={y_cam:.2f} Z={z_cam:.2f} "
                      f"→ Servo: Base={servo_angles[0]}° Arm1={servo_angles[1]}° "
                      f"Arm2={servo_angles[2]}° Gripper={servo_angles[3]}°")

    def send_servos(self):
        if self.pi_socket:
            cmd = {
                'type': 'servo_control',
                'positions': {f'servo{i}': self.servo_positions[i] for i in range(4)},
                'speed': self.speed
            }
            try:
                self.pi_socket.sendall((json.dumps(cmd) + '\n').encode())
            except:
                pass

    def closeEvent(self, event):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait()
        if self.pi_webcam_thread:
            self.pi_webcam_thread.stop()
            self.pi_webcam_thread.wait()
        if self.pi_socket:
            self.pi_socket.close()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotArmController()
    window.show()
    sys.exit(app.exec_())