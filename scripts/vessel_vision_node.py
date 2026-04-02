#!/usr/bin/env python3
import rospy
import cv2
import os
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vessel_detector import VesselAnalyzer 

class VesselVisionNode:
    def __init__(self):
        rospy.init_node('vessel_vision_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_p = os.path.join(script_dir, '..', 'models', 'best.pt')
        
        self.analyzer = VesselAnalyzer(model_path=model_p)
        
        self.ratio_pub = rospy.Publisher('/vessel/score', Float32, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Vision Node Online. Monitoring /camera/image_raw...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result = self.analyzer.process_frame(frame)
            score = result['score']
            self.ratio_pub.publish(result['score'])
            rospy.loginfo(f"Vessel Score: {score:.3f}")
            # Show live feed with YOLO overlay
            cv2.imshow("Vessel Detection", result['frame'])
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Vision Node Error: {e}")

if __name__ == '__main__':
    try:
        node = VesselVisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()