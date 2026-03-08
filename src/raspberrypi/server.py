import socket
import json
import time
import serial
import cv2
import pickle
import struct
import threading

# Open serial connection to Arduino
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    timeout=1
)

time.sleep(2)  # Wait for Arduino to initialize

def move_arm(base, arm1, arm2, gripper, speed):
    """
    Move robot arm to specified positions
    
    Physical Servo Ranges:
        base: 110-250 degrees (110=left, 250=right)
        arm1: 70-140 degrees (70=horizontal/down, 140=vertical/up)
        arm2: 80-130 degrees (80=straight/extended, 130=bent down)
        gripper: 90-160 degrees (90=closed, 160=open)
        speed: 1-100 (1=slow, 100=fast)
    """
    command = f"{base},{arm1},{arm2},{gripper},{speed}\n"
    print(f"Sending: {command.strip()}")
    
    # Clear any old data in buffer
    ser.reset_input_buffer()
    
    # Send command
    ser.write(command.encode())
    
    # Wait for responses
    time.sleep(0.1)
    while ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        print(f"Arduino: {response}")

def webcam_server():
    """
    Separate server for streaming webcam feed
    Runs on port 8001

    Optimized for low latency with aggressive frame dropping
    """
    print("\n[WEBCAM] Starting webcam server on port 8001...")

    video_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    video_server.bind(('0.0.0.0', 8001))
    video_server.listen(5)

    print("[WEBCAM] Webcam server ready, waiting for connections...")

    while True:
        try:
            client_socket, addr = video_server.accept()
            print(f"[WEBCAM] Client connected from {addr}")

            # Optimize socket for low latency
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Smaller send buffer to prevent frame accumulation
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 32768)

            # Start webcam capture
            cap = cv2.VideoCapture(0)

            # Optimize camera settings
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                print("[WEBCAM] Error: Could not open webcam")
                client_socket.close()
                continue

            print("[WEBCAM] Webcam opened, streaming...")

            # Frame timing control
            last_frame_time = time.time()
            target_fps = 30
            frame_interval = 1.0 / target_fps

            try:
                while True:
                    # Grab multiple frames and only process the latest
                    # This flushes the camera buffer
                    for _ in range(2):  # Grab 2 frames, keep only the last
                        ret = cap.grab()
                    
                    ret, frame = cap.retrieve()
                    
                    if not ret:
                        print("[WEBCAM] Failed to capture frame")
                        break

                    # Frame rate limiting
                    current_time = time.time()
                    elapsed = current_time - last_frame_time
                    if elapsed < frame_interval:
                        time.sleep(frame_interval - elapsed)
                    last_frame_time = time.time()

                    # Reduce resolution for lower bandwidth (optional)
                    # frame = cv2.resize(frame, (480, 360))  # Uncomment if still choppy

                    # Lower JPEG quality for smaller size and faster encoding
                    _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    data = encoded.tobytes()

                    # Send message size and frame data
                    message_size = struct.pack("Q", len(data))
                    try:
                        client_socket.sendall(message_size + data)
                    except:
                        print("[WEBCAM] Client disconnected")
                        break

            except Exception as e:
                print(f"[WEBCAM] Error during streaming: {e}")
            finally:
                cap.release()
                client_socket.close()
                print("[WEBCAM] Client disconnected, webcam released")

        except Exception as e:
            print(f"[WEBCAM] Connection error: {e}")

# Start webcam server in separate thread
webcam_thread = threading.Thread(target=webcam_server, daemon=True)
webcam_thread.start()

# Create socket server for servo control
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 8000))
server.listen(1)

print("=" * 50)
print("SERVO SERVER - Running on port 8000")
print("Waiting for connections...")
print("=" * 50)

# ============================================================================
# Current positions with middle-of-range defaults (SAFE STARTUP POSITION)
# ============================================================================

current_positions = {
    'base': 140,      # Middle of 110-250 range (centered)
    'arm1': 105,     # Middle of 70-140 range (mid-height)
    'arm2': 105,     # Middle of 80-130 range (mid-position)
    'gripper': 125   # Middle of 90-160 range (half-open)
}

print("\nDefault starting positions:")
print(f"  Base:    {current_positions['base']}° (range: 110-180)")
print(f"  Arm1:    {current_positions['arm1']}° (range: 70-140°)")
print(f"  Arm2:    {current_positions['arm2']}° (range: 80-130°)")
print(f"  Gripper: {current_positions['gripper']}° (range: 90-160°)")
print("=" * 50)

# ============================================================================
# Servo validation ranges (MATCHES CLIENT SPECIFICATION)
# ============================================================================

SERVO_RANGES = {
    'base': (110, 180),       # Base rotation: 0° (left) to 90° (right)
    'arm1': (70, 140),     # Rear arm: 70° (horizontal) to 140° (vertical)
    'arm2': (80, 130),     # Forearm: 80° (straight) to 130° (bent down)
    'gripper': (90, 160)   # Gripper: 90° (closed) to 160° (open)
}

def validate_position(servo_name, value):
    """
    Validate and clamp servo position to valid range
    
    This ensures that no matter what the client sends, the servos will
    never be commanded to positions outside their physical limits.
    
    Args:
        servo_name: Name of servo ('base', 'arm1', 'arm2', 'gripper')
        value: Requested servo angle (degrees)
    
    Returns:
        Clamped value within valid range
    """
    min_val, max_val = SERVO_RANGES.get(servo_name, (0, 180))
    clamped = max(min_val, min(max_val, int(value)))
    
    if clamped != value:
        print(f"⚠ WARNING: {servo_name} position {value}° clamped to {clamped}°")
    
    return clamped

# ============================================================================
# Main server loop
# ============================================================================

while True:
    try:
        client, addr = server.accept()
        print(f"\n✓ Connected to {addr}")
        print("-" * 50)
        
        buffer = ""
        while True:
            data = client.recv(1024).decode('utf-8')
            if not data:
                print(f"✗ Client {addr} disconnected")
                break
            
            buffer += data
            
            # Process complete JSON commands (separated by newlines)
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if not line.strip():
                    continue
                
                try:
                    print(f"\nReceived: {line}")
                    cmd = json.loads(line)
                    
                    # ========================================================
                    # Command processing with validation
                    # ========================================================
                    
                    if cmd['type'] == 'servo_control':
                        positions = cmd['positions']
                        print(f"Command type: servo_control")
                        print(f"Raw positions: {positions}")
                        
                        # Map servo0-3 to base, arm1, arm2, gripper
                        servo_map = {
                            'servo0': 'base',
                            'servo1': 'arm1',
                            'servo2': 'arm2',
                            'servo3': 'gripper'
                        }
                        
                        # Update current positions with validation
                        for servo_key, arm_key in servo_map.items():
                            if servo_key in positions:
                                raw_value = positions[servo_key]
                                validated_value = validate_position(arm_key, raw_value)
                                current_positions[arm_key] = validated_value
                        
                        # Get speed from command or use default
                        speed = cmd.get('speed', 50)  # Default speed: 50
                        speed = max(1, min(100, speed))  # Clamp speed to 1-100
                        
                        print(f"Validated positions: {current_positions}")
                        print(f"Speed: {speed}")
                        
                        # Send command to Arduino
                        move_arm(
                            base=current_positions['base'],
                            arm1=current_positions['arm1'],
                            arm2=current_positions['arm2'],
                            gripper=current_positions['gripper'],
                            speed=speed
                        )
                                
                    print("-" * 50)
                    
                except json.JSONDecodeError as e:
                    print(f"✗ JSON Error: {e}")
                except Exception as e:
                    print(f"✗ Error processing command: {e}")
                    
    except KeyboardInterrupt:
        print("\n\nShutting down server...")
        break
    except Exception as e:
        print(f"✗ Connection error: {e}")
    finally:
        try:
            client.close()
        except:
            pass

ser.close()
server.close()
print("Server stopped.")
