"""Keep test traffic off the robot's DDS domain; runs before any rclpy.init."""
import multiprocessing
import os

os.environ['ROS_DOMAIN_ID'] = str(1 + os.getpid() % 100)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

# ament_flake8 starts worker processes; forking a process that runs ROS threads can hang.
multiprocessing.set_start_method('forkserver', force=True)
