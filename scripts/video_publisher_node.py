#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_video():
    rospy.init_node('video_publisher_node')
    # Use the same topic your vision_node is listening to
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    
    # Replace with the path to your .mp4 or .avi file
    video_path = 'vessel_ultrasound.mp4' 
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Could not open video file!")
        return

    # Match the video's original speed (e.g., 30 FPS)
    rate = rospy.Rate(30) 

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # Convert OpenCV frame to ROS Image message
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(msg)
        else:
            # Loop the video if it ends
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass