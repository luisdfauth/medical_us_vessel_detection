import cv2
from ultralytics import YOLO

model = YOLO("models/best.pt")
cap = cv2.VideoCapture("MRP-2025-11-04/videos/TEST 2 VIDEO/20251017 054351.mp4")


while cap.isOpened():
    success, frame = cap.read()
    if success:
        results = model(frame)
        
        annotated_frame = results[0].plot()

        cv2.imshow("Vessel Tracker", annotated_frame)
        
    else:
        break

cap.release()
cv2.destroyAllWindows()

