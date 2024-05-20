import asyncio
import numpy as np
import threading
import offboard_setup
import function
import time

from scipy.optimize import least_squares, minimize
from mavsdk.offboard import (VelocityNedYaw)

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

async def initial_movement(uavs, drone_lat_long, tracker_coordinate, relative_distances, coordinates, initial_guess, uwb_info):
    # 一開始找目標位置-------------------------------------------------------------------------
    
    uav_tracker = uavs[0]
    distance = 0.0
    print("-------Start intial movement--------")
    # UWB distance
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance0 ={uwb_dist}")
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.01)
    
    z_intial = round(tracker_coordinate[-1][2], 3)
    # 往x, y, z方向移動-------------------------------------------------------------------------
    print("Tracker go up 1 m/s")
    await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1.0, 0))
    velocity_ned_yaw = VelocityNedYaw(0.0, 0.0, -1.0, 0)
    await asyncio.sleep(1)

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.01)

    # UWB distance
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance1 ={relative_distances[-1]}")
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.2)
    move_z = tracker_coordinate[-1][2] - tracker_coordinate[-2][2]

    print("Tracker go North 1 m/s")
    # 2023.12.19-----------------------------------------------------
    await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(1.0, 0.0, 0.0, 0))
    velocity_ned_yaw = VelocityNedYaw(1.0, 0.0, 0.0, 0)
    await asyncio.sleep(1)

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.2)

    # UWB distance
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance2 ={relative_distances[-1]}")
    # -----------------------------------------------------------------------------------------------
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.01)
    move_y = tracker_coordinate[-1][1] - tracker_coordinate[-2][1]
    # -----------------------------------------------------------------------------------------------

    print("Tracker go East 1 m/s")
    # 2023.12.19-----------------------------------------------------
    await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(0.0, 1.0, 0.0, 0))
    velocity_ned_yaw = VelocityNedYaw(0.0, 1.0, 0.0, 0)
    await asyncio.sleep(1)
    # UWB distance
    uwb_dist = get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance3 ={relative_distances[-1]}")

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.1)
    # -----------------------------------------------------------------------------------------------
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.1)

    move_x = tracker_coordinate[-1][0] - tracker_coordinate[-2][0]
    # -----------------------------------------------------------------------------------------------
    # 朝目標飛行
    # 找目標座標
    initial_guess[2] = function.z_guess(
        relative_distances[-4], relative_distances[-3], move_z, z_intial)
    diff_z = initial_guess[2] - tracker_coordinate[-1][0]
    initial_guess[0], initial_guess[1] = function.initial_target(
        relative_distances[-3], relative_distances[-2], relative_distances[-1], move_y, move_x, diff_z, diff_z, diff_z, tracker_coordinate)
    print(f"guess target position: {initial_guess}")

    print("====================================================")
    print("Tracker position: ", tracker_coordinate)
    print("distance: ", relative_distances)
