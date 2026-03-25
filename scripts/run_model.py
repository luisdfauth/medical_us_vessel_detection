import cv2
from vessel_detector import VesselAnalyzer

analyzer = VesselAnalyzer(model_path='models/best.pt')

#use this for simulation
video_path = "MRP-2025-11-04/videos/TEST 2 VIDEO/20251017 054351.mp4"
cap = cv2.VideoCapture(video_path)
##

#use this for real use
# cap = cv2.VideoCapture(0)
##


while cap.isOpened():
    success, frame = cap.read()
    
    if success:
        result = analyzer.process_frame(frame)
        
        status = result["status"]
        annotated_img = result["frame"]
        score = result["score"]
        
        print(f"Robot Input -> State: {status} | Ratio: {result['ratio']:.2f} | Score: {score}")

        cv2.imshow("Ultrasound Simulation", annotated_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("End of video.")
        break

cap.release()
cv2.destroyAllWindows()