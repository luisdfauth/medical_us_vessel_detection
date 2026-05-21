#!/home/cty/anaconda3/envs/yolo_env/bin/python
# -*- coding: utf-8 -*-

import sys
# Must be placed before import cv2 to prevent the Python3 environment from loading ROS Kinetic's Python2 OpenCV
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except ValueError:
    pass

import cv2
import numpy as np
import time
from ultralytics import YOLO

class VesselAnalyzer:
    def __init__(self, model_path):
        # Keep path explicitly passed to ensure the model can be accurately located when ROS starts
        print("[YOLO] Loading model: {}".format(model_path))
        self.model = YOLO(model_path, task='segment')

        # Print the currently used device to confirm that the environment has loaded the GPU correctly
        print("[YOLO] The current model is running on the device: {}".format(self.model.device))
        print("[YOLO] Model loaded successfully!")

    def process_frame(self, frame):
        # ------------------ Core Optimization Area ------------------
        # imgsz=480: Reduce inference size to stay within the 2GB VRAM limit of the GTX 960
        # device=0: Force inference on GPU
        # verbose=False: Disable terminal printing to prevent frame rate drops
        results = self.model(frame, verbose=False, imgsz=480, device=0)
        annotated_frame = frame.copy()

        # Check if masks were detected
        if not results[0].masks:
            return {
                "status": "NO_VESSEL",
                "width": 0,
                "height": 0,
                "ratio": 0,
                "score": 0.0,
                "frame": annotated_frame,
                "center": (0, 0)
            }

        try:
            # Extract the boundary points of the first mask and convert to int32
            mask_coords = results[0].masks.xy[0].astype(np.int32)
            
            # Fault tolerance: If there are fewer than 3 points, the minimum bounding rectangle cannot be calculated; treat as not detected
            if len(mask_coords) < 3:
                raise ValueError("Not enough points in mask")
        except Exception:
            return {
                "status": "NO_VESSEL",
                "width": 0,
                "height": 0,
                "ratio": 0,
                "score": 0.0,
                "frame": annotated_frame,
                "center": (0, 0)
            }
        
        # Draw the contour of the vessel (green)
        cv2.drawContours(annotated_frame, [mask_coords], -1, (0, 255, 0), 2)

        # Calculate the minimum area rectangle
        rect = cv2.minAreaRect(mask_coords)
        center, (w, h), angle = rect

        # Differentiate between the long axis and the short axis
        width = min(w, h)
        length = max(w, h)
        
        # ======= New Logic: Get image dimensions to determine if it spans the full width =======
        frame_h, frame_w = frame.shape[:2]
        is_full_width = length >= (frame_w * 0.85)   # 95% threshold for "full"
        
        if width == 0: 
            aspect_ratio = 0
        else:
            aspect_ratio = length / width

        # Threshold settings
        TRANSVERSE_MAX = 1.2
        LONGITUDINAL_MIN = 5.3
        LONGITUDINAL_MAX = 6.5

        # State determination and scoring logic
        if 0.8 < aspect_ratio < TRANSVERSE_MAX:
            state = "TRANSVERSE (Target Reached)"
            color = (0, 255, 0) # Green
            score = 1.0
            
        # ======= New Logic: Add is_full_width condition =======
        elif LONGITUDINAL_MIN < aspect_ratio < LONGITUDINAL_MAX and is_full_width: 
            state = "LONGITUDINAL (Target Reached)"
            color = (0, 0, 255) # Red
            score = 10.0
            
        else:
            input_range = LONGITUDINAL_MIN - TRANSVERSE_MAX 
            # Linear mapping formula to calculate scores during rotation
            score = 1.0 + (aspect_ratio - TRANSVERSE_MAX) * (9.0 / input_range)
            state = "ROTATING..."
            color = (0, 255, 255) # Yellow

        # Safety constraint: Ensure the score is always in the [0.0, 10.0] range to prevent robotic arm malfunction due to anomalies
        score = max(0.0, min(10.0, score))
        score = round(score, 2)

        # Get and draw the four vertices of the rectangle
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(annotated_frame, [box], 0, color, 2)
        
        # Add text information to the image
        cv2.putText(annotated_frame, "Ratio: {:.2f}".format(aspect_ratio), (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(annotated_frame, "Score: {:.2f}".format(score), (20, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(annotated_frame, state, (20, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Debugging assistance: Print the Full Width status on the screen for observation
        cv2.putText(annotated_frame, "Full Width: {}".format(is_full_width), (20, 130), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Return a dictionary containing all rich features
        return {
            "status": state,
            "width": width,
            "height": length,
            "ratio": aspect_ratio,
            "frame": annotated_frame,
            "center": center,
            "score": score
        }

# ================= New: PID Controller Class =================
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=10.0, max_output=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.max_output = max_output
        self.min_output = -max_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def compute(self, current_score):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0: dt = 1e-4 
            
        error = self.setpoint - current_score
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        output = p_term + i_term + d_term
        
        # Anti-integral windup
        if output > self.max_output:
            output = self.max_output
            self.integral -= error * dt
        elif output < self.min_output:
            output = self.min_output
            self.integral -= error * dt
            
        self.prev_error = error
        self.prev_time = current_time
        return output, error
