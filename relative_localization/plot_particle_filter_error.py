#!/usr/bin/env python

import sys
import os

import rosbag

sys.path.append(os.path.realpath('../../ga_slam_experiment_bags'))
filename = 'relative_localization/global_and_slam_paths_1_particle.bag'

# node = 'ga_slam_visualization'
# topics = ['global_pose_path', 'slam_pose_path']

paths = list()

with rosbag.Bag(filename, 'r') as bag:
    for topic, path_msg, timestamp in bag.read_messages():
        path = list()

        for pose_msg in path_msg.poses:
            position = pose_msg.pose.position
            path.append((position.x, position.y))

        paths.append(path)

if len(paths) != 2: exit('Expected 2 paths...')

print(paths)

