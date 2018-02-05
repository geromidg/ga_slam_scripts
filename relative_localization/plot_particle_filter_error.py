#!/usr/bin/env python

import sys
import os

from math import hypot

from matplotlib import pyplot as plt

from rosbag import Bag

bag_dir = '../../ga_slam_experiment_bags/'
sys.path.append(os.path.relpath(bag_dir))
filename = bag_dir + 'relative_localization/global_and_slam_paths_1_particle.bag'

path_1_topic = '/ga_slam_visualization/global_pose_path'
path_2_topic = '/ga_slam_visualization/slam_pose_path'

path_1_msg = None
path_2_msg = None

with Bag(filename, 'r') as bag:
    for topic, path_msg, _ in bag.read_messages():
        if topic == path_1_topic:
            path_1_msg = path_msg
        elif topic == path_2_topic:
            path_2_msg = path_msg

def convert_path_msg_to_list(path_msg):
    path = list()

    for pose_msg in path_msg.poses:
        position = pose_msg.pose.position
        path.append((position.x, position.y))

    return path

path_1 = convert_path_msg_to_list(path_1_msg)
path_2 = convert_path_msg_to_list(path_2_msg)

if len(path_1) != len(path_2): exit('Path sizes do not match!')

errors = list()

for i, pose_1 in enumerate(path_1):
    pose_2 = path_2[i]
    error = hypot(pose_1[0] - pose_2[0], pose_1[1] - pose_2[1])
    errors.append(error * 100.)

figure = plt.figure(figsize=(10, 6))

axes = figure.add_subplot(1, 1, 1)
axes.set_xlabel('Number of samples', fontsize=18)
axes.set_ylabel('Pose Error [cm]', fontsize=18)
axes.plot(errors, '-', linewidth=2, markersize=20, color="blue")
axes.margins(0.04)

figure.tight_layout()
figure.subplots_adjust(top=0.88)

plt.show()
# figure.savefig('error.png', dpi=60, bbox_inches='tight')

