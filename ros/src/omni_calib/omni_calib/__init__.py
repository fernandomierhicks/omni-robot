"""omni-robot calibration (design.md 10): the solvers and checks behind the calibration workflow (README.md). No ROS in
the cores: the sim's tests feed them synthetic detections with a known truth; on the robot they take OpenCV's /
AprilTag's detections (detect.py) from recorded sessions."""
