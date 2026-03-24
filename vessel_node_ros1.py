import rospy
from std_msgs.msg import Float32, String
import cv2
from vessel_detector import VesselAnalyzer 

def talker():
    # 1. Initialize ROS 1 Node
    rospy.init_node('vessel_vision_node', anonymous=True)
    
    ratio_pub = rospy.Publisher('/vessel/ratio', Float32, queue_size=10)
    
    analyzer = VesselAnalyzer(model_path='models/best.pt')
    cap = cv2.VideoCapture(0) # Or video path

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        success, frame = cap.read()
        if success:
            result = analyzer.process_frame(frame)
            
            # Publish the data
            ratio_pub.publish(result['ratio'])
            
            # Show for debugging
            cv2.imshow("Ultrasound Stream", result['frame'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass