# Drone-Tracking

This is an implementation for a drone tracking system using a camera and a Raspberry Pi to acquire the images. The tracking system is intended to use the YOLOv11 to identify the drones in some frames and the Extended Kalman Filter algorithm to keep tracking of the drone in the other frames or if the camera doesn't have the drone in the capture. This system is being developed to act as a tracker in a UGV-UAV hybrid collaboration, where the UGV is intended to serve as a mobile ground station for the UAV during extensive inspection missions.

## To do list:
- Clean the Raspberry Pi to improve performance ✅
- Implement the camera ✅
- Use YOLOv11 to detect drones with the camera ✅
- Retrain the network to improve drone detection ❌
- Implement the EKF algorithm ❌
- Implement the position map to build the drone position history ❌
- Implement the motor controller to track the drone ❌
- Finish building the UGV ❌
- Build the UAV landing platform on the UGV ❌
