import rospy
import threading
import function
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
# Assuming the class is saved in a module named drone_controller
from mavros_setup import DroneController
import mavros_setup
import time


def get_uwb_dist(uwb_info):
    print("Get uwb data....")
    uwb_err = 0
    uwb_dist = uwb_info.get_module_data()
    if uwb_dist == None:
        while uwb_dist == None:
            if uwb_err >= 80:
                print("UWB data err........")
                uwb_dist = 9999
                return uwb_dist
            uwb_dist = uwb_info.get_module_data()
            uwb_err += 1
            time.sleep(0.025)
    uwb_dist = float(uwb_dist)
    return uwb_dist


def initial_movement(drones, drone_lat_long, target_coordinate, tracker_coordinate, relative_distances, coordinates, initial_guess, uwb_info):
    uav_tracker, uav_target = drones
    distance = 0.0
    # 確保無人機已解鎖並設為OFFBOARD模式
    if not uav_tracker.arm():
        rospy.logerr("Failed to arm the drone.")
        return
    rospy.sleep(1)  # 稍等一秒鐘以確保無人機已解鎖
    if not uav_tracker.set_mode("OFFBOARD"):
        rospy.logerr("Failed to set OFFBOARD mode.")
        return

    print("-------Start initial movement--------")
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance0 ={uwb_dist}")
    mavros_setup.add_coordinates(uav_tracker, tracker_coordinate)
    z_intial = round(tracker_coordinate[-1][2], 3)

    print("Tracker go Up 1 m/s")
    uav_tracker.set_velocity(0.0, 0.0, 1.0, duration=1)
    rospy.sleep(1)
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance1 ={uwb_dist}")
    mavros_setup.add_coordinates(uav_tracker, tracker_coordinate)
    move_z = tracker_coordinate[-1][2] - tracker_coordinate[-2][2]
    rospy.logerr(f"move Z = {move_z}")

    print("Tracker go North 1 m/s")
    uav_tracker.set_velocity(0.0, 1.0, 0.0, duration=1)
    rospy.sleep(1)
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance2 ={uwb_dist}")
    mavros_setup.add_coordinates(uav_tracker, tracker_coordinate)
    move_y = tracker_coordinate[-1][1] - tracker_coordinate[-2][1]
    rospy.logerr(f"move Y = {move_y}")

    print("Tracker go East 1 m/s")
    uav_tracker.set_velocity(1.0, 0.0, 0.0, duration=1)
    rospy.sleep(1)
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance3 ={uwb_dist}")
    mavros_setup.add_coordinates(uav_tracker, tracker_coordinate)
    move_x = tracker_coordinate[-1][0] - tracker_coordinate[-2][0]
    rospy.logerr(f"move X = {move_x}")

    uav_tracker.set_velocity(0.0, 0.0, 0.0, 1.0)  # Stop the UAV

    # 找目標座標
    initial_guess = [0, 0, 0]
    initial_guess[2] = function.z_guess(
        relative_distances[-4], relative_distances[-3], move_z, z_intial)
    diff_z = initial_guess[2] - tracker_coordinate[-1][0]
    initial_guess[0], initial_guess[1] = function.initial_target(
        relative_distances[-3], relative_distances[-2], relative_distances[-1], move_y, move_x, diff_z, diff_z, diff_z, tracker_coordinate)
    print(f"guess target position: {initial_guess}")

    print("====================================================")
    print("Tracker position: ", tracker_coordinate)
    print("distance: ", relative_distances)
