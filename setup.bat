@echo off
if not exist "yolo_env" python -m venv yolo_env
call yolo_env\Scripts\activate
python -m pip install --upgrade pip
pip uninstall opencv-python-headless -y
pip install ultralytics opencv-python numpy
pause
