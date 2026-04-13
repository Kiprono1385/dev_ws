#Launch the model using camera feed
cd ~/Documents/potato_disease_detection
source install/setup.bash
ros2 launch potato_disease_detection model.launch.py

#View frames only
cd ~/Documents/potato_disease_detection
source install/setup.bash
ros2 run potato_disease_detection camera_viewer