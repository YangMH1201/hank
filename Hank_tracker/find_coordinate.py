import asyncio
import numpy as np
import function
import initial_move
from scipy.optimize import least_squares, minimize
from mavsdk.offboard import VelocityNedYaw
import time
import csv
import datetime

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


async def start_mission(uavs, drone_lat_long, uwb_info):
    uav_tracker = uavs[0]
    interval = 0.1
    # initialize paramters
    tracker_coordinate = []
    relative_distances = []
    coordinates = []
    initial_guess = [0.0, 0.0, 0.0]
    past_target_coordinates = []  # 存儲過去的target_coordinates

    velocity_distance = 1
    stop_distance = 2.0
    number = 0
    stop_number = 0
    count = 0

    # find the initial tracking velocity
    # initial_velocity = await function.calculate_initial_velocity(drone_lat_long)
    initial_velocity = 1.0
    # Start intiial movement to find the target
    await initial_move.initial_movement(uavs, drone_lat_long, tracker_coordinate, relative_distances, coordinates, initial_guess, uwb_info)

    distance = relative_distances[-1]

    while True:

        if distance < stop_distance:
            stop_number += 1
        else:
            stop_number = 0

        if stop_number >= 5:
            save_data_to_csv_with_timestamp(tracker_coordinate, relative_distances, past_target_coordinates)
            break  # 如果距离小于停止距离，则结束循环
        if distance >=20:
            break
        # target_position = estimate_3d_target(initial_guess, tracker_coordinate[:], relative_distances[:])
        target_position = gradient_descent_method(
            initial_guess, tracker_coordinate[-10:], relative_distances[-10:])
        velocity = PD_velocity(
            relative_distances, initial_velocity, velocity_distance)

        if number > 10:
            # 限制速度在最小和最大值之間
            velocity = max(0.0, min(velocity, 1.0))
            target_position = map_estimation(
                past_target_coordinates[-10:], target_position, tracker_coordinate[-5:], relative_distances)
            Vx, Vy, Vz = follow_me(
                tracker_coordinate[-1], target_position, velocity, relative_distances[-1], velocity_distance)
        else:
            # 限制速度在最小和最大值之間
            velocity = max(0.0, min(velocity, 1.0))
            target_position = mle_method(
                target_position, tracker_coordinate[-5:], relative_distances)
            Vx, Vy, Vz = follow_me(
                tracker_coordinate[-1], target_position, velocity, relative_distances[-1], velocity_distance)
            # Vz = max(-0.2, min(0.2, Vz))
        print(f"--------------------------------------------------")
        print(f"predict target_coordinate: {target_position}")

        print(f"velocity: {velocity}, Vx: {Vx}, Vy: {Vy}, Vz: {Vz}")
        await uav_tracker.offboard.set_velocity_ned(VelocityNedYaw(Vy, Vx, -Vz, 0))
        await asyncio.sleep(interval)

        # -----------------------------------------------------------------------------------------------
        # UWB distance
        # await function.distance_data(relative_distances)
        relative_distances.append(get_uwb_dist(uwb_info))
        distance = relative_distances[-1]
        print(f"distance={distance}")

        if isinstance(target_position, np.ndarray):
            target_position = target_position
        past_target_coordinates.append(target_position)
        number += 1

        predict_distance = np.linalg.norm(
            np.array(tracker_coordinate[-1]) - np.array(target_position))

        # 检查条件是否满足，并更新计数器
        if predict_distance < distance:
            count += 1
            if count == 5:
                number = 0  # 重置number为0
                count = 0  # 重置计数器
        else:
            count = 0  # 如果条件不满足，则重置计数器


def estimate_3d_target(initial_guess, drone_positions, relative_distances):
    """
    使用最小二乘法估算三维目标位置。

    参数：
    drone_positions：无人机当前位置的列表，每个位置是一个包含[x, y, z]的列表。
    relative_distances：相对距离数据的列表，每个距离是一个数值。

    返回值：
    estimated_target_position：估计的目标位置，一个包含[x, y, z]的列表。
    """
    def objective_function(target_position, drone_positions, relative_distances):
        """
        目标函数：将估计的目标位置与相对距离数据进行比较，返回残差。
        """
        residuals = []
        for i in range(len(drone_positions)):
            drone_position = drone_positions[i]
            d_relative = relative_distances[i]
            dx = drone_position[0] - target_position[0]
            dy = drone_position[1] - target_position[1]
            dz = drone_position[2] - target_position[2]
            predicted_distance = np.sqrt(dx**2 + dy**2 + dz**2)
            residuals.append(predicted_distance - d_relative)
        return residuals

    # 使用最小二乘法拟合目标位置，指定使用 LM 算法
    result = least_squares(objective_function, initial_guess, args=(
        drone_positions, relative_distances), method='lm')

    # 获取估计的目标位置
    if result.success:
        estimated_target_position = result.x
    else:
        print(f"Optimization failed: {result.message}")
        estimated_target_position = None  # 或者您可以返回一个特殊值或引发异常，具体取决于您的需求

    return estimated_target_position


def PD_velocity(distance, current_velocity, stop_distance):
    kp = 0.5
    kd = 0.01

    current_distance = distance[-1]
    last_distance = distance[-2]
    # 計算誤差
    error = current_distance - stop_distance

    # 計算誤差的變化率（這裡簡化為當前速度）
    error_derivative = current_distance - last_distance
    # error_derivative = current_velocity

    # 使用PD控制公式計算新速度
    # velocity = kp * error + kd * error_derivative
    velocity = kp * error - kd * error_derivative

    # 如果當前距離小於或等於停止距離，則停止
    if current_distance <= stop_distance:
        velocity = 0.0

    return velocity


def follow_me(current_position, target_position, speed, uwb_distance, distance_to_stop):

    if target_position is None:
        print("Target position is None. Handling error...")
        return

    # Calculate the deltas
    delta_x = target_position[0] - current_position[0]
    delta_y = target_position[1] - current_position[1]
    delta_z = target_position[2] - current_position[2]

    # Check for very small delta to avoid division by zero
    if uwb_distance < 1e-6:
        return 0.0, 0.0, 0.0

    # If the distance is less than the stopping distance, stop
    if uwb_distance < distance_to_stop:
        return 0.0, 0.0, 0.0

    # Calculate the velocity components
    velocity_x = (delta_x / uwb_distance) * speed
    velocity_y = (delta_y / uwb_distance) * speed
    velocity_z = (delta_z / uwb_distance) * speed

    # If the drone is very close to the target, reduce the speed
    if uwb_distance < speed:
        velocity_x *= (uwb_distance / speed)
        velocity_y *= (uwb_distance / speed)
        velocity_z *= (uwb_distance / speed)

    # velocity_z = max(-0.5, min(0.5, velocity_z))

    return velocity_x, velocity_y, velocity_z

# Method to find the target coordinate


def gradient_descent_method(initial_guess, drone_positions, relative_distances):
    def objective_function(target_position):
        residuals = []
        for drone_position, d_relative in zip(drone_positions, relative_distances):
            distance = np.linalg.norm(
                np.array(drone_position) - np.array(target_position))
            residuals.append(distance - d_relative)
        return np.sum(np.square(residuals))

    # # 约束条件: Z 值不小于 0 且不大于 10
    # constraints = (
    #     {'type': 'ineq', 'fun': lambda x: x[2]},  # x[2] 表示 Z 坐标，不小于 0
    #     {'type': 'ineq', 'fun': lambda x: 10 - x[2]}  # Z 坐标不大于 10
    # )

    # 调整优化器参数
    options = {'maxiter': 1000, 'disp': False}  # 增加迭代次数，关闭显示
    # result = minimize(objective_function, initial_guess, method='SLSQP', constraints=constraints, options=options)
    result = minimize(objective_function, initial_guess,
                      method='SLSQP', options=options)

    if result.success:
        return result.x
    else:
        print(f"Optimization failed: {result.message}")
        return None

# 2023/12/29 Maximum a Posteriori Estimation


def calculate_prior(prior_target_coordinates, relative_distances, target_position):
    # 計算每個座標點的權重，根據距離是否減少
    prior_target_coordinates = [np.array(coord) if isinstance(
        coord, list) else coord for coord in prior_target_coordinates]

    weights = [1 if i > 0 and relative_distances[i] <
               relative_distances[i - 1] else 0 for i in range(len(relative_distances))]
    weighted_coords = np.array(
        [w * coord for w, coord in zip(weights, prior_target_coordinates)], dtype=object)
    weighted_coords = [coord if isinstance(coord, np.ndarray) else np.zeros_like(
        prior_target_coordinates[0]) for coord in weighted_coords]
    # weighted_coords = np.array([w * coord for w, coord in zip(weights, prior_target_coordinates)])

    # 更新均值和方差計算，考慮權重
    # mean = np.sum(weighted_coords, axis=0) / np.sum(weights) if np.sum(weights) > 0 else np.mean(prior_target_coordinates, axis=0)
    # variance = np.var(weighted_coords, axis=0) if len(weighted_coords) > 1 else np.ones(3)
    mean = np.mean(prior_target_coordinates, axis=0)
    variance = np.var(prior_target_coordinates, axis=0) if len(
        prior_target_coordinates) > 1 else np.ones(3)
    # 計算先驗值
    prior = np.exp(-0.5 * np.sum((target_position - mean)**2 / variance))
    return prior


def map_estimation(past_target_coordinates, current_guess, drone_positions, relative_distances):
    min_length = min(len(past_target_coordinates), len(relative_distances))
    relative_distances1 = relative_distances[-min_length:]
    min_length = min(len(drone_positions), len(relative_distances))
    relative_distances2 = relative_distances[-min_length:]

    def posterior(target_position):
        prior = calculate_prior(past_target_coordinates,
                                relative_distances1, target_position)
        likelihood = np.product([np.exp(-0.5 * (np.linalg.norm(np.array(drone_position) - target_position) - d_relative)**2)
                                for drone_position, d_relative in zip(drone_positions, relative_distances2)])
        return -prior * likelihood

    # Z-axis constraint
    constraints = ({'type': 'ineq', 'fun': lambda x: x[2]})

    # 尝试使用 'COBYLA' 方法
    result = minimize(posterior, current_guess,
                      method='COBYLA', constraints=constraints)
    # result = minimize(posterior, current_guess, method='BFGS')
    if result.success:
        # 手动确保 Z 轴非负
        optimized_position = result.x
        if optimized_position[2] <= 0:
            optimized_position[2] = current_guess[2]
        return optimized_position
        # return result.x
    else:
        print(f"Optimization failed: {result.message}")
        return current_guess

# 2024/03/17 Maximum Likelihood Estimation


def mle_method(initial_guess, drone_positions, relative_distances, noise_variance=1.0):
    def likelihood_function(target_position):
        log_likelihood = 0
        for drone_position, d_relative in zip(drone_positions, relative_distances):
            distance = np.linalg.norm(
                np.array(drone_position) - np.array(target_position))
            # 高斯噪聲下的似然函數，這裡使用對數似然來避免乘積下溢
            log_likelihood += -0.5 * np.log(2 * np.pi * noise_variance) - (
                (distance - d_relative) ** 2) / (2 * noise_variance)
        # 最大化似然函數等同於最小化負的似然函數
        return -log_likelihood

    options = {'maxiter': 1000, 'disp': False}
    result = minimize(likelihood_function, initial_guess,
                      method='L-BFGS-B', options=options)

    if result.success:
        return result.x
    else:
        print(f"Optimization failed: {result.message}")
        return None

def save_data_to_csv_with_timestamp(tracker_coordinate, relative_distances, past_target_coordinates):
    # Get the current date and time
    now = datetime.datetime.now()
    # Format the date and time as a string in the format 'YYYY-MM-DD_HH-MM-SS'
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    # Create a filename using the current date and time
    filename = f"uav_tracking_data_{timestamp}.csv"

    # Open the file in write mode
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)

        # Write the header
        writer.writerow(["Tracker Coordinate", "Relative Distances", "Past Target Coordinates"])

        # Assuming all lists are of the same length
        for i in range(len(tracker_coordinate)):
            writer.writerow([tracker_coordinate[i], relative_distances[i], past_target_coordinates[i]])

    print(f"Data saved to {filename}")

