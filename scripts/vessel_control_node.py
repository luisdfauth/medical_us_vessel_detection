#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float64

class VesselControlNode:
    def __init__(self):
        rospy.init_node('vessel_control_node', anonymous=True)
        
        # 1. PID Parameters initialization (Requires tuning)

        self.kp = 0.5  
        self.ki = 0.05 
        self.kd = 0.1  
        
        # 2. Setpoint initialization
        # ideal ratio
        #add map from 1-10
        self.setpoint = 5
        
        # 3. State variables for PID
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = rospy.Time.now()
        
        # Anti-windup limit for the integral term
        self.i_limit = 2.0 
        
        # 4. Publishers
        # Publishes the control action signal (joint velocity command)
        self.action_pub = rospy.Publisher('/robot/joint_velocity_cmd', Float64, queue_size=10)
        
        # 5. Subscribers
        # Subscribes to the live vessel ratio from the vision node
        self.ratio_sub = rospy.Subscriber('/vessel/score', Float32, self.ratio_callback)

        
        rospy.loginfo("Control Node Online. Waiting for target ratio and live ratio data...")

    def target_callback(self, msg):
        """Updates the dynamic setpoint based on high-level logic or user input."""
        self.setpoint = msg.data
        rospy.loginfo(f"New target ratio received: {self.setpoint:.3f}")

    def ratio_callback(self, msg):
        # Halt control loop if the target ratio is still unknown
        if self.setpoint is None:
            rospy.logwarn_throttle(2.0, "Waiting for target ratio... Control paused.")
            return

        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        
        # Prevent division by zero on the first callback or if time hasn't advanced
        if dt <= 0.0:
            self.prev_time = current_time
            return
            
        current_ratio = msg.data
        

        # Discrete-Time PID Control Law Implementation

        
        # 1. Calculate error e[k]
        error = self.setpoint - current_ratio
        
        # 2. Proportional term P[k] = Kp * e[k]
        p_out = self.kp * error
        
        # 3. Integral term I[k] = I[k-1] + e[k] * dt
        self.integral += error * dt
        # Integral anti-windup
        self.integral = max(min(self.integral, self.i_limit), -self.i_limit)
        i_out = self.ki * self.integral
        
        # 4. Derivative term D[k] = (e[k] - e[k-1]) / dt
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative
        
        # 5. Calculate final control effort u[k]
        control_effort = p_out + i_out + d_out
        
        
        # Update states for the next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        # Publish the action signal to the robot's low-level velocity controller
        self.action_pub.publish(control_effort)
        
        # Log for debugging
        rospy.loginfo(f"Target: {self.setpoint:.3f} | Live: {current_ratio:.3f} | Error: {error:.3f} | Cmd Vel: {control_effort:.3f} rad/s")

if __name__ == '__main__':
    try:
        node = VesselControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
