import asyncio
import numpy as np
import threading
import offboard_setup
import function
from uwb_read import UwbModule
import time

from scipy.optimize import least_squares, minimize
from mavsdk.offboard import (VelocityNedYaw)

asyncdef get_uwb_dist(uwb_info):
        uwb_err = 0
        uwb_dist = uwb_info.get_module_data()
        if uwb_dist == None:
            while uwb_dist == None:
                if uwb_err >= 80:
                    print("UWB data err........")
                    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
                    await asyncio.sleep(5)
                    await offboard_setup.stop_offboard_mode(uavs)
                    await uavs[0].action.land()
                    raise
                uwb_dist = uwb_info.get_module_data()
                uwb_err += 1
                time.sleep(0.05)
        uwb_dist = float(uwb_dist)

        return uwb_dist

async def initial_movement(uavs, drone_lat_long, tracker_coordinate, relative_distances, coordinates, initial_guess):
    # 一開始找目標位置-------------------------------------------------------------------------
    uwb_info = UwbModule()
    uav_tracker = uavs[0]
    distance = 0.0

    print("-------Start intial movement--------")
    # UWB distance
    uwb_dist = await get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance0 ={uwb_dist}")
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    # function.add_coordinates(coordinates, drone_lat_long)
    await asyncio.sleep(0.01)
    
    z_intial = round(tracker_coordinate[-1][2], 3)
    # 往x, y, z方向移動-------------------------------------------------------------------------
    print("Tracker go up 1 m/s")
    # await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1.0, 0))
    # await move_uav(uav_tracker, VelocityNedYaw(0.0, 0.0, -1.0, 0), 1.0)
    # 2023.12.19-----------------------------------------------------
    velocity_ned_yaw = VelocityNedYaw(0.0, 0.0, -1.0, 0)
    # tracker_move_task = move_uav(uav_tracker, velocity_ned_yaw, 1.0)
    # await tracker_move_task

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.01)

    # UWB distance
    uwb_dist = await get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance1 ={relative_distances[-1]}")
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.2)
    # function.add_coordinates(coordinates, drone_lat_long)
    move_z = tracker_coordinate[-1][2] - tracker_coordinate[-2][2]

    print("Tracker go North 1 m/s")
    # 2023.12.19-----------------------------------------------------
    velocity_ned_yaw = VelocityNedYaw(1.0, 0.0, 0.0, 0)
    await asyncio.sleep(1)
    # tracker_move_task = move_uav(uav_tracker, velocity_ned_yaw, 1.0)
    # await tracker_move_task

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.2)

    # UWB distance
    uwb_dist = await get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance2 ={relative_distances[-1]}")
    # -----------------------------------------------------------------------------------------------
    await offboard_setup.local_position(uav_tracker, tracker_coordinate)
    await asyncio.sleep(0.01)
    # function.add_coordinates(coordinates, drone_lat_long)
    move_y = tracker_coordinate[-1][1] - tracker_coordinate[-2][1]
    # -----------------------------------------------------------------------------------------------

    print("Tracker go East 1 m/s")
    # await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    # await asyncio.sleep(0.1)
    # await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(0.0, 1.0, 0.0, 0.0))
    # await move_uav(uav_tracker, VelocityNedYaw(0.0, 1.0, 0.0, 0), 1.0)
    # 2023.12.19-----------------------------------------------------
    velocity_ned_yaw = VelocityNedYaw(0.0, 1.0, 0.0, 0)
    await asyncio.sleep(1)
    # UWB distance
    uwb_dist = await get_uwb_dist(uwb_info)
    relative_distances.append(uwb_dist)
    print(f"distance3 ={relative_distances[-1]}")
    # -----------------------------------------------------------------------------------------------
    # tracker_move_task = move_uav(uav_tracker, velocity_ned_yaw, 1.0)
    # await tracker_move_task

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
    # initial_guess = gradient_descent_method(initial_guess, tracker_coordinate[-4:], relative_distances[-4:])
    print(f"guess target position: {initial_guess}")

    print("====================================================")
    print("Tracker position: ", tracker_coordinate)
    print("distance: ", relative_distances)


async def move_uav(uav, velocity_ned_yaw, duration):
    # 发送移动指令
    await uav.offboard.set_velocity_ned(velocity_ned_yaw)
    # 等待指定的时间（根据速度和距离计算）
    await asyncio.sleep(duration)
    # 停止无人机（设置速度为0）
    await uav.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0))
    # 检查无人机是否停止
    while not await check_uav_stopped(uav.telemetry):
        await asyncio.sleep(0.01)


async def check_uav_stopped(telemetry):
    VELOCITY_THRESHOLD = 0.1  # 速度阈值
    async for velocity_ned in telemetry.velocity_ned():
        is_stopped = abs(velocity_ned.north_m_s) < VELOCITY_THRESHOLD and \
            abs(velocity_ned.east_m_s) < VELOCITY_THRESHOLD and \
            abs(velocity_ned.down_m_s) < VELOCITY_THRESHOLD
        if is_stopped:
            return True


def gradient_descent_method(initial_guess, drone_positions, relative_distances):
    def objective_function(target_position):
        residuals = []
        for drone_position, d_relative in zip(drone_positions, relative_distances):
            distance = np.linalg.norm(
                np.array(drone_position) - np.array(target_position))
            residuals.append(distance - d_relative)
        return np.sum(np.square(residuals))

    result = minimize(objective_function, initial_guess,
                      method='BFGS')  # BFGS是一种梯度下降算法

    if result.success:
        return result.x
    else:
        print(f"Optimization failed: {result.message}")
        return None


def run_async_in_thread(func, *args):
    # 在這個函數中創建一個新的事件循環
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    # 創建並執行協程
    coroutine = func(*args)
    loop.run_until_complete(coroutine)
    loop.close()
