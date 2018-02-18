#!/usr/bin/env python

import sys
import os

from math import hypot, sqrt

import numpy as np

from matplotlib import pyplot as plt
from matplotlib2tikz import save as tikz_save

from rosbag import Bag

plt.style.use('ggplot')

def read_path_msgs_from_bag(filename, path_1_topic, path_2_topic):
    path_1_msg = None
    path_2_msg = None

    with Bag(filename, 'r') as bag:
        for topic, path_msg, _ in bag.read_messages():
            if topic == path_1_topic:
                path_1_msg = path_msg
            elif topic == path_2_topic:
                path_2_msg = path_msg

    return path_1_msg, path_2_msg

def convert_path_msg_to_list(path_msg):
    path = list()

    for pose_msg in path_msg.poses:
        position = pose_msg.pose.position
        path.append((position.x, position.y))

    return path

def calculate_path_error(path_1, path_2):
    if len(path_1) == 0: exit('Path is empty!')
    if len(path_1) != len(path_2): exit('Path sizes do not match!')

    path_error = list()

    for i, pose_1 in enumerate(path_1):
        pose_2 = path_2[i]
        error = hypot(pose_1[0] - pose_2[0], pose_1[1] - pose_2[1])
        path_error.append(error * 100.)

    return path_error

def calculate_mse(path_error):
    sum = 0.0
    for error in path_error: sum += error * error

    return sum / len(path_error)

def calculate_rmsd(path_error):
    mse = calculate_mse(path_error)
    sum = 0.0
    for error in path_error: sum += (error - mse) * (error - mse)

    return sqrt(sum / len(path_error))

def plot_number_of_particles(particles_20, particles_100, particles_200):
    plt.plot(particles_20, '-k', label='20 particles', linewidth=4)
    plt.plot(particles_100, '-r', label='100 particles', linewidth=4)
    plt.plot(particles_200, '-c', label='200 particles', linewidth=4)
    plt.xlabel('Time (s)', fontsize=15)
    plt.ylabel('Pose Error (cm)', fontsize=15)
    plt.title('Parameter Comparison: Number of Particles', fontsize=18)
    plt.legend(loc='best', fontsize='medium', shadow=True, framealpha=0.5)
    plt.grid(True)
    plt.show()

    tikz_save('error_vs_particles.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth')

def plot_resampling_frequency(resampling_1, resampling_5, resampling_20):
    plt.plot(resampling_1, '-k', label='1 iteration', linewidth=4)
    plt.plot(resampling_5, '-r', label='5 iterations', linewidth=4)
    plt.plot(resampling_20, '-c', label='20 iterations', linewidth=4)
    plt.xlabel('Time (s)', fontsize=18)
    plt.ylabel('Pose Error (cm)', fontsize=18)
    plt.title('Parameter Comparison: Resampling Frequency')
    plt.legend(loc='best', fontsize='medium', shadow=True, framealpha=0.5)
    plt.grid(True)
    plt.show()

    tikz_save('error_vs_resampling.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth')

if __name__ == '__main__':
    bag_dir = '../ga_slam_experiment_bags/'
    sys.path.append(os.path.relpath(bag_dir))
    path_1_topic = '/ga_slam_visualization/global_pose_path'
    path_2_topic = '/ga_slam_visualization/slam_pose_path'

    filename = bag_dir + 'relative_localization/global_and_slam_paths_1_particle.bag'
    path_1_msg, path_2_msg = read_path_msgs_from_bag(filename, path_1_topic, path_2_topic)
    path_1 = convert_path_msg_to_list(path_1_msg)
    path_2 = convert_path_msg_to_list(path_2_msg)
    particles_20 = calculate_path_error(path_1, path_2)

    filename = bag_dir + 'relative_localization/global_and_slam_paths_2_particle.bag'
    path_1_msg, path_2_msg = read_path_msgs_from_bag(filename, path_1_topic, path_2_topic)
    path_1 = convert_path_msg_to_list(path_1_msg)
    path_2 = convert_path_msg_to_list(path_2_msg)
    particles_100 = calculate_path_error(path_1, path_2)

    filename = bag_dir + 'relative_localization/global_and_slam_paths_3_particle.bag'
    path_1_msg, path_2_msg = read_path_msgs_from_bag(filename, path_1_topic, path_2_topic)
    path_1 = convert_path_msg_to_list(path_1_msg)
    path_2 = convert_path_msg_to_list(path_2_msg)
    particles_200 = calculate_path_error(path_1, path_2)

    plot_number_of_particles(particles_20, particles_100, particles_200)
    # plot_resampling_frequency(resampling_1, resampling_5, resampling_20)

