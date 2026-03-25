from ultralytics import YOLO
import cv2
import numpy as np

class VesselAnalyzer:
    def __init__(self, model_path='models/best.pt'):
        self.model = YOLO(model_path)

    def process_frame(self, frame):

        results = self.model(frame, verbose=False)
        
        annotated_frame = frame.copy()

        if not results[0].masks:
            return {
                "status": "NO_VESSEL",
                "width": 0,
                "height": 0,
                "ratio": 0,
                "score": 0,
                "frame": annotated_frame
            }

        mask_coords = results[0].masks.xy[0].astype(np.int32)
        
        cv2.drawContours(annotated_frame, [mask_coords], -1, (0, 255, 0), 2)

        rect = cv2.minAreaRect(mask_coords)
        (center), (w, h), angle = rect

        width = min(w, h)
        length = max(w, h)
        
        if width == 0: 
            aspect_ratio = 0
        else:
            aspect_ratio = length / width

        TRANSVERSE_MAX = 1.2
        LONGITUDINAL_MIN = 5.3
        LONGITUDINAL_MAX = 6.5

        if 0.8 < aspect_ratio < TRANSVERSE_MAX:
            state = "TRANSVERSE (Target Reached)"
            color = (0, 255, 0) # Green
            score = 1
        elif LONGITUDINAL_MIN< aspect_ratio < LONGITUDINAL_MAX: 
            state = "LONGITUDINAL (Target Reached)"
            color = (0, 0, 255) # Red
            score = 10

        else:
            input_range = LONGITUDINAL_MIN - TRANSVERSE_MAX 
            score = 1.0 + (aspect_ratio - TRANSVERSE_MAX) * (9.0 / input_range)
            
            state = "ROTATING..."
            color = (0, 255, 255) # Yellow

        # Round score for cleaner reading
        score = round(score, 2)

        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(annotated_frame, [box], 0, color, 2)
        
        # Draw text
        cv2.putText(annotated_frame, f"Ratio: {aspect_ratio:.2f}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(annotated_frame, state, (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return {
            "status": state,
            "width": width,
            "height": length,
            "ratio": aspect_ratio,
            "frame": annotated_frame,
            "center": center,
            "score": score
        }