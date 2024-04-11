#!/usr/bin/env python3
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""

class Location:
    def __init__(self, id_loc, x, y) -> None:
        self.x = x
        self.y = y
        self.id_loc = id_loc
        self.objects = dict()
    
    def add_item(self, name, obj_quant):
        if name in self.objects:
            self.objects[name] += obj_quant
        else:
            self.objects[name] = obj_quant
    
    def del_item(self, name, obj_quant):
        assert name in self.objects and self.objects[name] - obj_quant < 0
        self.objects[name] -= obj_quant      
   
def menu():
    while True:
        print("Add location: 1")
        print("Add object: 2")
        print("Deliver: 3")
        choice = input("Enter your choice: ") 
        if int(choice) == 1 or int(choice) == 2 or int(choice) == 3:
            return int(choice) 
        print("Invalid choice")

def print_locations(location_list):
    for location in location_list:
        print(location.id_loc, end = ', ')

def print_location_objs(selected_location):
    for name in selected_location.objects:
        print(name, end=', ')

def set_destination(x,y, navigator):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0
    return goal_pose

def navigation_status(navigator):
    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.15
    initial_pose.pose.position.y = 0.10
    initial_pose.pose.orientation.z = 0.075
    # initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    location_list = []

    while 1:
        option = menu()

        if (option == 1):
            user_input = input("Enter location ID, x, y: ")
            loc_ID, pos_x, pos_y = user_input.split(',')
            location_list.append(Location(int(loc_ID), float(pos_x), float(pos_y)))
        
        if (option == 2):
            print("Select location: ", end=' ')
            print_locations(location_list)
            print()
            user_input_loc = input("ID of the location: ")

            selected_location = location_list[int(user_input_loc)-1]
            print_location_objs(selected_location)
            user_input_obj, user_input_quant = input("Name of the object and quantity: ").split(',')

            selected_location.add_item(user_input_obj, user_input_quant)

        if (option == 3):
            print("Select location for pick up: ", end=' ')
            print_locations(location_list)
            print()
            user_pickup_loc = input("ID of the location: ")

            print("Select object: ", end=' ')
            selected_location = location_list[int(user_pickup_loc)-1]
            print_location_objs(selected_location)
            print()
            user_pickup_obj, user_pickup_quant = input("Name of the object and quantity: ").split(',')
            # while (selected_location.objects[user_pickup_obj] - int(user_pickup_quant) < 0):
            #     user_pickup_quant = input("Enter lower quantity: ")

            delivery_x, delivery_y = input("Indicate delivery destination x,y: ").split(',')

            goal_pose_pickup = set_destination(selected_location.x, selected_location.y, navigator)
            navigator.goToPose(goal_pose_pickup)
            navigation_status(navigator)
            
            goal_pose_deliver = set_destination(float(delivery_x), float(delivery_y), navigator)
            navigator.goToPose(goal_pose_deliver)
            navigation_status(navigator)


    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()