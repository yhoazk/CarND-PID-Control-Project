#!/usr/bin/env python

# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error
#
# You'll only need to modify the `run` function at the bottom.
# ------------

import random
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------
#
# this is the Robot class
#

class Robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)

############## ADD / MODIFY CODE BELOW ####################
# ------------------------------------------------------------------------
#
# run - does a single control run
robot = Robot()
robot.set(0, 1, 0)

def run(robot, tau, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    steer = 0
    # TODO: your code here
    for x in range(n):
        x_trajectory.append(robot.x)
        print("X: " + str(robot.x))
        y_trajectory.append(robot.y)
        print("Y: " + str(robot.y))
        steer = 50000* tau * (0-robot.y) # proportional to the distance to y = 0
        print("Steer: "  + str(steer))
        robot.move(steer, 0.1)

x_trajectory, y_trajectory = run(robot, 0.1)
n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.show()


"""
Not quite. Your trajectory =

[(0, 1), (1.9999685891290879, 1.0097072950279937), (3.9997471793779571, 1.0389230845282498), (5.9991358608604024, 1.0880248132355916), (7.9979099799458417, 1.1577728871525608), (9.9958030187834055, 1.2493218411278804), (11.992486470110689, 1.364239185525463), (13.987545134173612, 1.5045322701488999), (15.980445892315279, 1.6726836870153363), (17.97049745959103, 1.8716959380915483), (19.956797820666473, 2.1051463306046969), (21.938164904552025, 2.3772533454474143), (23.913044386528234, 2.6929560730845026)]

[(1.9999685891290826, 0.99029270497200628), (3.9997502475180164, 0.9612665365966393), (5.9991677680707198, 0.91330230427888637), (7.9980670644827141, 0.84715527482057951), (9.9963296004381093, 0.76394268872519433), (11.993882235464632, 0.66512373126505508), (13.990703678090849, 0.55247259357929579), (15.9868270041807, 0.4280454212901077), (17.982338005843346, 0.29414203680437367), (19.97736945689509, 0.15326334026542554), (21.972091689031444, 0.0080652573597035371), (23.966705873238055, -0.13861213067878547), (25.961406488947659, -0.28410733290729695)] n = 13 speed = 2 tau = 0.09677050663527742 Starting from (0, 1, 0)
"""
