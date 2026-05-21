#!/home/cty/anaconda3/envs/yolo_env/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import numpy as np

# ================= Core Exorcism Code (Restore your original logic) =================
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    try:
        sys.path.remove(ros_path)
    except ValueError:
        pass

# 1. Without interference from the ROS path, safely load third-party libraries that tend to conflict
import cv2
from vessel_detector import VesselAnalyzer, PIDController

# 2. After loading cv2 and YOLO, the ROS path must be added back! Otherwise, rospy cannot be found!
sys.path.append(ros_path)
# ===================================================================================

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32, String
# Note: Completely abandon the official CvBridge and use our own Numpy Hack method

class VesselVisionNode:
    def __init__(self):
        rospy.init_node('vessel_vision_node', anonymous=True)
        
        # Automatic model path compatibility
        model_path = rospy.get_param('~model_path', '')
        if not model_path:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            model_path = os.path.join(script_dir, '..', 'models', 'best.pt')
            
        self.analyzer = VesselAnalyzer(model_path)
        
        # ================= Safety Modification Point 1 =================
        # Initialize PID controller (lower Kp and maximum output angle to ensure smooth and safe movement)
        self.pid = PIDController(kp=0.8, ki=0.01, kd=0.1, setpoint=10.0, max_output=2.0)
        self.rotation_direction = 1 
        self.last_error = 10.0      
        
        # New: Specialized variables for anti-high-frequency oscillation
        self.wrong_dir_counter = 0     # Continuous error counter
        self.smoothed_score = None     # Filtered/smoothed score
        # ================================================================

        # Publishers
        self.info_pub = rospy.Publisher('/vessel/info', Float32MultiArray, queue_size=1)
        self.cmd_pub = rospy.Publisher('/vessel_vision/rotation_cmd', Float32, queue_size=1)
        self.image_pub = rospy.Publisher('/vessel_vision/annotated_image', Image, queue_size=1)
        self.status_pub = rospy.Publisher('/vessel_vision/status', String, queue_size=1)

        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/frame_grabber/image_cropped_topic', 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("Vision Node Online. Waiting for ultrasound images...")

    # ==== Restore your god-tier Hack to bypass cv_bridge conflicts ====
    def imgmsg_to_cv2_hack(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return img

    # ==== New Reverse Hack for publishing images ====
    def cv2_to_imgmsg_hack(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = cv_image.shape[1] * 3
        img_msg.data = cv_image.tobytes()
        return img_msg

    def image_callback(self, msg):
        try:
            # 1. Pure Numpy image parsing
            frame = self.imgmsg_to_cv2_hack(msg)
            
            # 2. YOLO detection
            result = self.analyzer.process_frame(frame)
            raw_score = result['score']
            center_x, center_y = result['center'] 
            status = result['status']
            annotated_frame = result['frame']
            
            # ================= 1. Publish legacy compatibility array =================
            array_msg = Float32MultiArray()
            array_msg.data = [float(center_x), float(center_y), float(raw_score)]
            self.info_pub.publish(array_msg)

            # ================= 2. Industrial-grade robust PID control =================
            cmd_angle = 0.0
            
            # [Core Optimization A] Exponential Moving Average (EMA): Eliminate score fluctuations caused by YOLO edge flicker
            if self.smoothed_score is None:
                self.smoothed_score = raw_score
            else:
                alpha = 0.2 # Smoothing coefficient: 20% weight for new data, 80% for history
                self.smoothed_score = alpha * raw_score + (1.0 - alpha) * self.smoothed_score

            if status == "NO_VESSEL":
                self.pid.reset()
                cmd_angle = 0.0
                self.wrong_dir_counter = 0
            elif self.smoothed_score >= 9.8:
                self.pid.reset()
                cmd_angle = 0.0
                # Throttle log output to prevent console flooding
                rospy.loginfo_throttle(2.0, "Target Reached! Holding position.")
            else:
                # Calculate PID using the smoothed score
                step_angle, current_error = self.pid.compute(self.smoothed_score)
                
                # [Core Optimization B] Continuity Confirmation Mechanism (Debounce)
                # Only suspect the direction is wrong if the error increases significantly
                if current_error > self.last_error + 0.05:
                    self.wrong_dir_counter += 1
                else:
                    self.wrong_dir_counter = 0
                    self.last_error = current_error # Only update reference error when the direction is correct
                
                # Only execute a direction reversal if the error increases for [5 consecutive frames]
                if self.wrong_dir_counter >= 5:
                    rospy.logwarn("Confirmed wrong direction! Reversing...")
                    self.rotation_direction *= -1
                    self.pid.reset()
                    self.wrong_dir_counter = 0
                    self.last_error = current_error

                # [Core Optimization C] Deadzone Limit
                # Filter out tiny arithmetic commands to prevent high-frequency micro-jitter in robotic arm gears
                raw_cmd = step_angle * self.rotation_direction
                if abs(raw_cmd) < 0.2:
                    cmd_angle = 0.0
                else:
                    cmd_angle = raw_cmd

            self.cmd_pub.publish(Float32(cmd_angle))
            # Display the smoothed score when publishing status for easier observation
            self.status_pub.publish(String("Status: {} | Smoothed Score: {:.2f} | Cmd: {:.2f} deg".format(
                status, self.smoothed_score, cmd_angle)))

            # ================= 3. Seamless publication of annotated images =================
            img_msg = self.cv2_to_imgmsg_hack(annotated_frame)
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr("Error in vision callback: %s" % str(e))

if __name__ == '__main__':
    try:
        node = VesselVisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
