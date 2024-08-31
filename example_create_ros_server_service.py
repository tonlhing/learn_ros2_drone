#!/usr/bin/env python

import rospy
from drone_trajectories.srv import ComputeTrajectory, ComputeTrajectoryResponse
import math

def handle_compute_trajectory(req):
    rospy.loginfo("Computing trajectory for points: x=%f, y=%f, z=%f", req.x, req.y, req.z)
    distance = math.sqrt(req.x ** 2 + req.y ** 2 + req.z ** 2)
    return ComputeTrajectoryResponse(distance)

def compute_trajectory_server():
    rospy.init_node('compute_trajectory_server')
    s = rospy.Service('compute_trajectory', ComputeTrajectory, handle_compute_trajectory)
    rospy.loginfo("Ready to compute trajectory.")
    rospy.spin()

if __name__ == "__main__":
    compute_trajectory_server()
