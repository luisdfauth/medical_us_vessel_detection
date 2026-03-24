models/best.pt is the vision recognition model
val/.. has the validation analysis of the final model
blood vessel recognition.v3i.yolov8/.. contains the test, train and validation sets
setup.bat starts the yolo environment and installs possible necessary packages
test.py and sanity_check.py are codes for development of the model
vessel_detector.py contains the VesselAnalizer class and the process_frame function within
vessel_vision_node.py is the ROS 1 ready code for the vessel recognition function
