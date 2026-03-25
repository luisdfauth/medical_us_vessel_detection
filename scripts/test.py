from ultralytics import YOLO

model = YOLO('models/best.pt')

model.val(data='blood vessel recognition.v3i.yolov8\data.yaml', split='test')