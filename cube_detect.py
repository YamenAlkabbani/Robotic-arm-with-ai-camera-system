#!/usr/bin/env python3
"""
🤖 Cube Detection & Robotic Arm Control System
Controls LX-16A servos via Arduino with inverse kinematics
"""

import cv2
import numpy as np
import time
import serial
import math
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import os

class CubeDetector:
    def __init__(self):
        """
        Initialize the cube detector
        """
        # Define color ranges in HSV
        self.color_ranges = {
            'blue': ([90, 50, 50], [120, 255, 255]),
            'black': ([0, 0, 0], [180, 255, 50]),
            'brown': ([5, 50, 20], [20, 200, 150]),  # Brown is a dark orange
            'yellow': ([20, 100, 100], [40, 255, 255])
        }
        self.colors = list(self.color_ranges.keys())
        self.min_area = 500  # Minimum contour area to consider
        self.confidence_threshold = 0.85  # 85% confidence threshold

    def detect_cubes(self, frame):
        """
        Detect cubes in the frame and return detected objects
        
        Args:
            frame: Input frame (BGR format)
            
        Returns:
            List of detected cubes with color, center, bounding box, and confidence
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_cubes = []
        
        for color, (lower, upper) in self.color_ranges.items():
            # Create mask for color range
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Morphological operations to remove noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.min_area:
                    # Get bounding box and center
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Calculate confidence based on fill ratio
                    bbox_area = w * h
                    confidence = area / bbox_area if bbox_area > 0 else 0
                    
                    # Only consider detections above confidence threshold
                    if confidence >= self.confidence_threshold:
                        detected_cubes.append({
                            'color': color,
                            'center': (center_x, center_y),
                            'bbox': (x, y, w, h),
                            'area': area,
                            'confidence': confidence
                        })
        
        # Sort by area (largest first) and return max 3 cubes
        detected_cubes.sort(key=lambda x: x['area'], reverse=True)
        return detected_cubes[:3]

class RoboticArmController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """Initialize robotic arm controller"""
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.is_connected = False
        
        # Arm physical parameters (in mm)
        self.base_height = 100    # Height of base above work surface
        self.shoulder_length = 150  # Length from shoulder to elbow
        self.elbow_length = 150    # Length from elbow to wrist
        self.wrist_length = 80     # Length from wrist to end effector
        
        # Servo IDs
        self.base_servo_id = 1
        self.shoulder_servo_id = 2
        self.elbow_servo_id = 3
        self.wrist_servo_id = 4
        
        # Servo angle limits (in degrees)
        self.base_limits = (0, 240)
        self.shoulder_limits = (0, 240)
        self.elbow_limits = (0, 240)
        self.wrist_limits = (0, 240)
        
        # Camera calibration
        self.camera_width = 640
        self.camera_height = 480
        self.pixels_per_mm = 5.0  # Camera scale factor
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(2)  # Wait for connection to establish
            self.is_connected = True
            print(f"✅ Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def send_servo_command(self, servo_id, position, move_time=1000):
        """Send servo command to Arduino"""
        if not self.is_connected:
            return False
        
        try:
            # Send command in format "MOVE:ID:POSITION:TIME"
            command = f"MOVE:{servo_id}:{position}:{move_time}\n"
            self.serial_connection.write(command.encode())
            print(f"🤖 Sent servo command: {command.strip()}")
            return True
        except Exception as e:
            print(f"❌ Failed to send servo command: {e}")
            return False
    
    def calculate_inverse_kinematics(self, x, y, z=0):
        """
        Calculate inverse kinematics for the robotic arm
        Returns base, shoulder, elbow, and wrist angles in servo units (0-1000)
        """
        # Convert pixel coordinates to real-world coordinates (mm)
        world_x = (x - self.camera_width/2) / self.pixels_per_mm
        world_y = (self.camera_height - y) / self.pixels_per_mm  # Flip y-axis
        world_z = z
        
        # Distance from base to target in horizontal plane
        r = math.sqrt(world_x**2 + world_y**2)
        d = math.sqrt(r**2 + (world_z - self.base_height)**2)
        
        # Calculate base angle (rotation)
        base_angle = math.atan2(world_y, world_x)
        
        # Calculate shoulder and elbow angles using law of cosines
        cos_shoulder = (self.shoulder_length**2 + d**2 - self.elbow_length**2) / (2 * self.shoulder_length * d)
        shoulder_angle = math.atan2(world_z - self.base_height, r) + math.acos(cos_shoulder)
        
        cos_elbow = (self.shoulder_length**2 + self.elbow_length**2 - d**2) / (2 * self.shoulder_length * self.elbow_length)
        elbow_angle = math.pi - math.acos(cos_elbow)
        
        # Wrist angle (keep end effector horizontal)
        wrist_angle = math.pi/2 - (shoulder_angle + elbow_angle)
        
        # Convert angles to servo units (0-1000 = 0-240°)
        def angle_to_servo(angle_rad, limits):
            angle_deg = math.degrees(angle_rad)
            # Normalize to servo range
            servo_value = int(1000 * (angle_deg - limits[0]) / (limits[1] - limits[0]))
            return max(0, min(1000, servo_value))
        
        base_servo = angle_to_servo(base_angle, self.base_limits)
        shoulder_servo = angle_to_servo(shoulder_angle, self.shoulder_limits)
        elbow_servo = angle_to_servo(elbow_angle, self.elbow_limits)
        wrist_servo = angle_to_servo(wrist_angle, self.wrist_limits)
        
        return base_servo, shoulder_servo, elbow_servo, wrist_servo
    
    def move_to_position(self, x, y):
        """Move arm to specified position"""
        if not self.is_connected:
            return False
        
        try:
            # Calculate inverse kinematics
            base, shoulder, elbow, wrist = self.calculate_inverse_kinematics(x, y)
            
            # Send commands to all servos
            self.send_servo_command(self.base_servo_id, base)
            self.send_servo_command(self.shoulder_servo_id, shoulder)
            self.send_servo_command(self.elbow_servo_id, elbow)
            self.send_servo_command(self.wrist_servo_id, wrist)
            
            print(f"🎯 Moved to position ({x}, {y})")
            return True
        except Exception as e:
            print(f"❌ Inverse kinematics error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial_connection:
            self.serial_connection.close()
            self.is_connected = False

class CubeDetectionGUI:
    def __init__(self):
        """Initialize the GUI application"""
        self.root = tk.Tk()
        self.root.title("🤖 Cube Detection & Robotic Arm Control")
        self.root.geometry("1100x800")  # Increased size for logo
        
        # Initialize components
        self.cube_detector = CubeDetector()
        self.arm_controller = RoboticArmController()
        
        # Video capture
        self.cap = None
        self.current_frame = None
        self.detected_cubes = []
        self.target_cube = None
        
        # Control variables
        self.is_running = False
        self.last_send_time = 0
        self.send_interval = 0.5  # Send position every 500ms
        self.logo_image = None
        
        # Position verification
        self.position_verification_start = 0
        self.verification_active = False
        self.stable_position = None
        self.verification_threshold = 20  # Pixel threshold for position stability
        
        # Setup GUI
        self.setup_gui()
        
        # Connect to Arduino
        self.connect_arduino()
        
        # Start video capture
        self.start_camera()
        
        # Load fixed RoboTronics logo
        self.load_robotics_logo()
    
    def load_robotics_logo(self):
        """Load the fixed RoboTronics logo"""
        logo_path = "/home/pi/cube_detector/RoboTronics logo.png"
        if os.path.exists(logo_path):
            try:
                # Load and resize logo
                logo = Image.open(logo_path)
                logo = logo.resize((200, 100), Image.LANCZOS)
                self.fixed_logo_image = ImageTk.PhotoImage(logo)
                
                # Create fixed logo label
                self.fixed_logo_label = ttk.Label(self.logo_frame_left, image=self.fixed_logo_image)
                self.fixed_logo_label.pack(side=tk.LEFT, padx=10)
                print(f"✅ Fixed logo loaded from {logo_path}")
            except Exception as e:
                print(f"❌ Failed to load fixed logo: {e}")
        else:
            print(f"⚠️ Fixed logo not found at {logo_path}")
    
    def load_logo(self):
        """Load an additional logo image"""
        file_path = filedialog.askopenfilename(
            title="Select Logo Image",
            filetypes=[("Image Files", "*.png *.jpg *.jpeg *.bmp *.gif")]
        )
        
        if file_path:
            try:
                # Load and resize logo
                logo = Image.open(file_path)
                logo = logo.resize((200, 100), Image.LANCZOS)  # Adjust size as needed
                self.logo_image = ImageTk.PhotoImage(logo)
                
                # Create logo label if it doesn't exist
                if not hasattr(self, 'additional_logo_label'):
                    self.additional_logo_label = ttk.Label(self.logo_frame_right, image=self.logo_image)
                    self.additional_logo_label.pack(side=tk.RIGHT, padx=10)
                else:
                    self.additional_logo_label.config(image=self.logo_image)
                
                print(f"✅ Additional logo loaded from {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load logo: {e}")
    
    def setup_gui(self):
        """Setup the GUI elements"""
        # Configure root grid to center everything
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
        # Main frame - centered in root
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure main frame grid
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.columnconfigure(2, weight=1)
        main_frame.rowconfigure(0, weight=0)
        main_frame.rowconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=0)
        main_frame.rowconfigure(3, weight=0)
        
        # Header frame for logo and title
        header_frame = ttk.Frame(main_frame)
        header_frame.grid(row=0, column=0, columnspan=3, pady=10, sticky="ew")
        
        # Configure header frame columns
        header_frame.columnconfigure(0, weight=1)
        header_frame.columnconfigure(1, weight=2)  # More weight for center
        header_frame.columnconfigure(2, weight=1)
        
        # Left logo frame (for fixed RoboTronics logo)
        self.logo_frame_left = ttk.Frame(header_frame)
        self.logo_frame_left.grid(row=0, column=0, sticky="w")
        
        # Title frame (center)
        title_frame = ttk.Frame(header_frame)
        title_frame.grid(row=0, column=1, sticky="nsew")
        
        # Title label - centered
        title_label = ttk.Label(title_frame, 
                              text="Cube Detection & Robotic Arm Control", 
                              font=('Arial', 16, 'bold'))
        title_label.pack(expand=True, pady=10)
        
        # Right logo frame (for additional logo)
        self.logo_frame_right = ttk.Frame(header_frame)
        self.logo_frame_right.grid(row=0, column=2, sticky="e")
        
        # Video display - centered with padding
        self.video_label = ttk.Label(main_frame)
        self.video_label.grid(row=1, column=0, columnspan=3, pady=10, sticky="nsew")
        
        # Control buttons - centered
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=2, column=0, columnspan=3, pady=10, sticky="ew")
        
        # Center the buttons horizontally
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        button_frame.columnconfigure(2, weight=1)
        button_frame.columnconfigure(3, weight=1)
        button_frame.columnconfigure(4, weight=1)
        
        self.start_btn = ttk.Button(button_frame, text="🚀 Start Detection", command=self.toggle_detection)
        self.start_btn.grid(row=0, column=0, padx=5)
        
        # Cube selection
        cube_frame = ttk.LabelFrame(button_frame, text="Select Cube")
        cube_frame.grid(row=0, column=1, padx=10)
        
        self.cube_var = tk.StringVar(value='Largest')
        ttk.Radiobutton(cube_frame, text="Largest Cube", 
                       variable=self.cube_var, value='Largest').pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(cube_frame, text="Manual Select", 
                       variable=self.cube_var, value='Manual').pack(side=tk.LEFT, padx=5)
        
        self.track_btn = ttk.Button(button_frame, text="🎯 Track Selected", command=self.set_target_cube)
        self.track_btn.grid(row=0, column=2, padx=5)
        
        # Arm control buttons
        arm_frame = ttk.LabelFrame(button_frame, text="Arm Control")
        arm_frame.grid(row=0, column=3, padx=10)
        
        self.home_btn = ttk.Button(arm_frame, text="🏠 Home Position", command=self.move_home)
        self.home_btn.pack(side=tk.LEFT, padx=5)
        
        # Logo button
        self.logo_btn = ttk.Button(button_frame, text="🖼️ Load Additional Logo", command=self.load_logo)
        self.logo_btn.grid(row=0, column=4, padx=5)
        
        # Status display - centered
        status_frame = ttk.Frame(main_frame)
        status_frame.grid(row=3, column=0, columnspan=3, pady=10, sticky="ew")
        
        # Configure status frame for centered content
        status_frame.columnconfigure(0, weight=1)
        status_frame.columnconfigure(1, weight=1)
        status_frame.columnconfigure(2, weight=1)
        
        # Status labels - centered in three columns
        self.detection_var = tk.StringVar(value="Detection: Stopped")
        detection_label = ttk.Label(status_frame, textvariable=self.detection_var, font=('Arial', 12, 'bold'))
        detection_label.grid(row=0, column=0, sticky="w")
        
        self.tracking_var = tk.StringVar(value="Tracking: None")
        tracking_label = ttk.Label(status_frame, textvariable=self.tracking_var)
        tracking_label.grid(row=0, column=1, sticky="w")
        
        self.servo_var = tk.StringVar(value="Arduino: Disconnected")
        servo_label = ttk.Label(status_frame, textvariable=self.servo_var)
        servo_label.grid(row=0, column=2, sticky="w")
        
        self.confidence_var = tk.StringVar(value="Confidence: --")
        confidence_label = ttk.Label(status_frame, textvariable=self.confidence_var)
        confidence_label.grid(row=1, column=0, sticky="w")
        
        self.position_var = tk.StringVar(value="Position: --")
        position_label = ttk.Label(status_frame, textvariable=self.position_var)
        position_label.grid(row=1, column=1, sticky="w")
        
        self.verification_var = tk.StringVar(value="Verification: --")
        verification_label = ttk.Label(status_frame, textvariable=self.verification_var)
        verification_label.grid(row=1, column=2, sticky="w")
    
    def connect_arduino(self):
        """Connect to Arduino"""
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', 'COM3', 'COM4', 'COM5']
        for port in ports:
            self.arm_controller.port = port
            if self.arm_controller.connect():
                self.servo_var.set(f"Arduino: Connected ({port})")
                return
        
        self.servo_var.set("Arduino: Connection Failed")
    
    def start_camera(self):
        """Start camera capture"""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Could not open camera")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.update_frame()
    
    def update_frame(self):
        """Update video frame"""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()
                
                # Run detection if enabled
                if self.is_running:
                    self.detected_cubes = self.cube_detector.detect_cubes(frame)
                    
                    # Draw detection results on frame
                    self.draw_detection_results(frame)
                    
                    # Update detection info
                    self.detection_var.set(f"🔍 Detected {len(self.detected_cubes)} cubes")
                    
                    # Track selected cube
                    self.track_cube(frame)
                else:
                    self.detection_var.set("⏸️ Detection paused")
                
                # Convert to PhotoImage and display
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_pil = Image.fromarray(frame_rgb)
                frame_pil = frame_pil.resize((640, 480))
                photo = ImageTk.PhotoImage(frame_pil)
                
                self.video_label.configure(image=photo)
                self.video_label.image = photo
        
        # Schedule next update
        self.root.after(33, self.update_frame)  # ~30 FPS
    
    def draw_detection_results(self, frame):
        """Draw detection results on frame with improved layout"""
        # Color mapping for visualization
        color_map = {
            'blue': (255, 0, 0),      # Blue in BGR
            'black': (0, 0, 0),       # Black
            'brown': (0, 75, 150),    # Brown
            'yellow': (0, 255, 255)   # Yellow
        }
        
        # Determine target cube if any
        target = None
        if self.target_cube is not None and self.detected_cubes:
            if self.target_cube == 'Largest':
                target = self.detected_cubes[0]
            else:
                # For manual selection, use first cube for now
                target = self.detected_cubes[0]
        
        for i, cube in enumerate(self.detected_cubes):
            color = cube['color']
            x, y, w, h = cube['bbox']
            cx, cy = cube['center']
            confidence = cube['confidence']
            
            # Draw bounding box with color
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_map[color], 2)
            
            # Draw center point
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Create information blocks with proper spacing
            info_y = y - 10
            is_target = (cube == target)
            
            # Draw cube index at top left
            index_text = f"Cube {i+1}"
            cv2.putText(frame, index_text, (x, y - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Draw color and confidence below index
            color_text = f"{color.capitalize()} ({confidence:.0%})"
            cv2.putText(frame, color_text, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_map[color], 2)
            
            # Draw coordinates at bottom of bounding box
            coord_text = f"({cx}, {cy})"
            cv2.putText(frame, coord_text, (x, y + h + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # If this is the tracked cube, add tracking indicator
            if is_target:
                # Draw green border around the cube
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                
                # Draw tracking indicator at top center of bounding box
                tracking_text = "TRACKING"
                text_size = cv2.getTextSize(tracking_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                text_x = x + (w - text_size[0]) // 2
                cv2.putText(frame, tracking_text, (text_x, y - 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    def is_position_stable(self, current_pos):
        """Check if position is stable within threshold"""
        if self.stable_position is None:
            return False
            
        # Calculate Euclidean distance
        dx = current_pos[0] - self.stable_position[0]
        dy = current_pos[1] - self.stable_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        return distance <= self.verification_threshold
    
    def track_cube(self, frame):
        """Track the selected cube and move arm to position"""
        if not self.detected_cubes:
            self.tracking_var.set("Tracking: No cubes detected")
            self.confidence_var.set("Confidence: --")
            self.position_var.set("Position: --")
            self.verification_var.set("Verification: --")
            self.verification_active = False
            return
            
        # Determine target cube
        if self.target_cube is None:
            # Default to largest cube
            target = self.detected_cubes[0]
        elif self.target_cube == 'Largest':
            target = self.detected_cubes[0]
        else:
            # Manual selection (not implemented in this version)
            target = self.detected_cubes[0]
        
        cx, cy = target['center']
        confidence = target['confidence']
        
        # Update tracking info
        self.tracking_var.set(f"Tracking: {target['color']} cube")
        self.confidence_var.set(f"Confidence: {confidence:.0%}")
        self.position_var.set(f"Position: ({cx}, {cy})")
        
        # Position verification logic
        current_time = time.time()
        current_pos = (cx, cy)
        
        if not self.verification_active:
            # Start new verification
            self.verification_active = True
            self.position_verification_start = current_time
            self.stable_position = current_pos
            self.verification_var.set(f"Verification: Starting (0/3s)")
            print(f"🔄 Starting position verification at ({cx}, {cy})")
        else:
            # Check if position is stable
            if self.is_position_stable(current_pos):
                # Position is stable - update verification status
                elapsed = current_time - self.position_verification_start
                self.verification_var.set(f"Verification: {elapsed:.1f}/3s")
                
                # Check if we've reached 3 seconds
                if elapsed >= 3:
                    # Verification complete - send position
                    self.verification_var.set("Verification: ✅ Complete")
                    self.send_position_to_arm(cx, cy, target)
                    self.verification_active = False
            else:
                # Position has changed - restart verification
                self.position_verification_start = current_time
                self.stable_position = current_pos
                self.verification_var.set("Verification: Restarted")
                print("🔄 Position changed, restarting verification")
    
    def send_position_to_arm(self, x, y, target):
        """Send position to arm after verification"""
        # Ask for confirmation before sending
        response = messagebox.askyesno(
            "Confirm Movement",
            f"Move arm to verified position ({x}, {y})?\n"
            f"Tracking: {target['color']} cube\n"
            f"Confidence: {target['confidence']:.0%}\n"
            "Position verified for 3 seconds"
        )
        
        if response:
            if self.arm_controller.move_to_position(x, y):
                self.last_send_time = time.time()
                messagebox.showinfo("Success", "Arm movement command sent!")
            else:
                messagebox.showerror("Error", "Failed to send movement command")
        else:
            print("Movement cancelled by user")
    
    def set_target_cube(self):
        """Set the cube to track"""
        self.target_cube = self.cube_var.get()
        self.tracking_var.set(f"Tracking: {self.target_cube} cube")
        self.verification_active = False  # Reset verification when changing target
    
    def move_home(self):
        """Move arm to home position"""
        if self.arm_controller.is_connected:
            # Ask for confirmation
            response = messagebox.askyesno("Confirm", "Move arm to home position?")
            if response:
                # Send home position to all servos
                self.arm_controller.send_servo_command(1, 500)  # Base
                self.arm_controller.send_servo_command(2, 500)  # Shoulder
                self.arm_controller.send_servo_command(3, 500)  # Elbow
                self.arm_controller.send_servo_command(4, 500)  # Wrist
                print("🤖 Arm moved to home position")
                messagebox.showinfo("Success", "Arm moved to home position")
    
    def toggle_detection(self):
        """Toggle detection on/off"""
        self.is_running = not self.is_running
        if self.is_running:
            self.start_btn.config(text="⏹️ Stop Detection")
        else:
            self.start_btn.config(text="🚀 Start Detection")
            self.target_cube = None
            self.tracking_var.set("Tracking: None")
            self.verification_var.set("Verification: --")
            self.verification_active = False
    
    def run(self):
        """Run the application"""
        try:
            self.root.mainloop()
        finally:
            if self.cap:
                self.cap.release()
            self.arm_controller.disconnect()
            cv2.destroyAllWindows()

def main():
    """Main function"""
    print("🚀 Starting Cube Detection & Robotic Arm Control System...")
    print("📋 Features:")
    print("  • Cube detection (blue, black, brown, yellow)")
    print("  • Confidence threshold (85% minimum)")
    print("  • Max 3 cubes detected")
    print("  • Inverse kinematics for arm positioning")
    print("  • Real-time position display")
    print("  • LX-16A servo control via Arduino")
    print("  • Position verification for 3 seconds")
    print("  • Fixed RoboTronics logo on left")
    print("  • Additional logo on right")
    print("  • Improved cube labeling with clear indicators")
    print()
    
    # Create and run application
    app = CubeDetectionGUI()
    app.run()

if __name__ == "__main__":
    main()
