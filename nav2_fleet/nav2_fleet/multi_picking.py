#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from random import randint
from geometry_msgs.msg import PoseStamped
from nav2_fleet.new_robot_navigator import NewBasicNavigator, NewNavigationResult

import rclpy
from rclpy.duration import Duration

n_robots = 2
n_shelves = 4
n_destinations = 4

# Shelf positions for picking
shelves = [
    [-3.829, -7.604],
    [-3.791, -3.287],
    [-3.791, 1.254],
    [-3.24, 5.861]]

# Shipping destination for picked products
shipping_destinations = [
    [-0.205, 7.403],
    [-0.073, -8.497],
    [6.217, 2.153],
    [-6.349, 9.147]]

def random_exclude(exclude):
    randInt = randint (0, n_shelves-1)
    return random_exclude(exclude) if randInt in exclude else randInt

def create_requests():
    exclude = []
    requests = []
    for i in range(n_robots):
        destination = randint(0, n_destinations-1)
        shelf = random_exclude(exclude)
        exclude.append(shelf)
        requests.append((shelf, destination))
    return requests

def new_request(robot,requests):
    exclude = []
    for i in range(n_robots):
        if i != robot :
            exclude.append(requests[i][0])
    destination = randint(0, n_destinations-1)
    shelf = random_exclude(exclude)
    requests[robot] = (shelf, destination)
    return requests

def create_pose(position, stamp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def main():
    #Received inital requests for picking items
    requests = create_requests()
    rclpy.init()

    navigators = [NewBasicNavigator(ns = 'robot{}'.format(i+1)) for i in range(n_robots)]
    picked = [False for i in range(n_robots)]

    # Set our demo's initial pose
    for i in range(n_robots):
        stamp = navigators[i].get_clock().now().to_msg()
        navigators[i].setInitialPose(create_pose(shipping_destinations[i], stamp))
        # Wait for navigation to fully activate
        navigators[i].waitUntilNav2Active()

    for i in range(n_robots):
        stamp = navigators[i].get_clock().now().to_msg()
        position = shelves[requests[i][0]]
        navigators[i].goToPose(create_pose(position, stamp))

    while True:
        for i in range(n_robots):
            if navigators[i].isNavComplete():
                #if navigators[i].getResult() == NewNavigationResult.SUCCEEDED:
                if True:
                    if picked[i]:
                        stamp = navigators[i].get_clock().now().to_msg()
                        position = shipping_destinations[requests[i][1]]
                        navigators[i].goToPose(create_pose(position, stamp))
                    else:
                        requests = new_request(i, requests)
                        stamp = navigators[i].get_clock().now().to_msg()
                        position = shelves[requests[i][0]]
                        navigators[i].goToPose(create_pose(position, stamp))
                    picked[i] = not picked[i]

if __name__ == '__main__':
    main()
