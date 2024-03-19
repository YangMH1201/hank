import asyncio
import numpy as np
import math
import offboard_setup
from lat_long_dist import lat_long_dist
import uwb_read

lat_long_distance = lat_long_dist()

# initail guess-----------------------------------------------------------------------------------------------------


def z_guess(distance1, distance2, move_rate, z_initial):
    z_guess = z_initial + (distance1**2-distance2**2 +
                           (move_rate)**2)/(2*(move_rate))
    print(f"z initail: {z_initial}")
    print(f"target z position: {z_guess}")
    return z_guess

# 利用固定點迭代, 兩次距離計算目標位置

# def fun(distance1, distance2, move_rate, angle):
#     up = (distance2**2 - distance1**2 + move_rate**2 + 2 * move_rate * distance2 *
#           (-angle**3 / math.factorial(3) + angle**5 / math.factorial(5) - angle**7 / math.factorial(7)))
#     down = -2 * move_rate * distance2
#     return up / down


def fun(distance1, distance2, move_rate, angle):
    try:
        up = (distance2**2 - distance1**2 + move_rate**2 +
              2 * move_rate * distance2 * math.sin(angle))
        down = -2 * move_rate * distance2

        if down == 0:
            return None
        return up / down
    except OverflowError:
        # Handle the overflow error, perhaps by logging it or setting a default value
        print("Overflow error occurred in fun function")
        return 0  # or some other default value


def fixed_point_iteration(distance1, distance2, move_rate, initial_guess=0, tolerance=1e-6, max_iterations=250):
    x = initial_guess
    iterations = 0
    last_x = x

    while iterations < max_iterations:
        if fun(distance1, distance2, move_rate, x) is None:
            x = None
            break
        next_x = fun(distance1, distance2, move_rate, x)
        if abs(next_x - x) < tolerance:
            break
        if iterations > 0 and abs(next_x - last_x) > abs(x - last_x):
            print("Warning: Possible divergence detected.")
            break
        last_x = x
        x = next_x
        iterations += 1

    return x

# 兩次固定點迭代計算目標位置


def initial_target(distance1, distance2, distance3, move1, move2, diff_z1, diff_z2, diff_z3, origin):

    # Ensures non-negative value
    d1 = math.sqrt(max(distance1**2 - diff_z1**2, 0))
    d2 = math.sqrt(max(distance2**2 - diff_z2**2, 0))
    d3 = math.sqrt(max(distance3**2 - diff_z3**2, 0))
    angle1 = fixed_point_iteration(d1, d2, move1)
    angle2 = fixed_point_iteration(d2, d3, move2)

    print((angle1, angle2))

    if origin[-2][0] == 0:
        heading_angle1 = math.pi / 2 if origin[-2][1] > 0 else -math.pi / 2
    else:
        heading_angle1 = math.atan(origin[-2][1] / origin[-2][0])

    if origin[-1][0] == 0:
        heading_angle2 = math.pi / 2 if origin[-1][1] > 0 else -math.pi / 2
    else:
        heading_angle2 = math.atan(origin[-1][1] / origin[-1][0])
    if angle1 is None:
        Rpre_abs_target = origin[-2][:2]
        Lpre_abs_target = origin[-2][:2]
    else:
        Rpre_targetx = + d2 * math.cos(angle1)
        Lpre_targetx = - d2 * math.cos(angle1)
        pre_targety = move1 + d2 * math.sin(angle1)
        Rpre_abs_target = rotate_point(
            [Rpre_targetx, pre_targety], heading_angle1, origin[-2][:2])
        Lpre_abs_target = rotate_point(
            [Lpre_targetx, pre_targety], heading_angle1, origin[-2][:2])
    if angle2 is None:
        R_abs_target = origin[-1][:2]
        L_abs_target = origin[-1][:2]
    else:
        Rtargetx = + d3 * math.cos(angle2)
        Ltargetx = - d3 * math.cos(angle2)
        targety = move2 + d3 * math.sin(angle2)
        R_abs_target = rotate_point(
            [Rtargetx, targety], heading_angle2, origin[-1][:2])
        L_abs_target = rotate_point(
            [Ltargetx, targety], heading_angle2, origin[-1][:2])

    dRR = math.sqrt((Rpre_abs_target[0]-R_abs_target[0])
                    ** 2+(Rpre_abs_target[1]-R_abs_target[1])**2)
    dLR = math.sqrt((Lpre_abs_target[0]-R_abs_target[0])
                    ** 2+(Lpre_abs_target[1]-R_abs_target[1])**2)
    dLL = math.sqrt((Lpre_abs_target[0]-L_abs_target[0])
                    ** 2+(Lpre_abs_target[1]-L_abs_target[1])**2)
    dRL = math.sqrt((Rpre_abs_target[0]-L_abs_target[0])
                    ** 2+(Rpre_abs_target[1]-L_abs_target[1])**2)

    # 找到最小距离的索引
    min_distance_index = min(range(4), key=lambda i: [dRR, dLR, dLL, dRL][i])

    # 根据最小距离的索引设置 targetx 和 targety 的值
    if min_distance_index <= 1:
        targetx = R_abs_target[0]
        targety = R_abs_target[1]
    else:
        targetx = L_abs_target[0]
        targety = L_abs_target[1]

    return targetx, targety
# end initail guess-----------------------------------------------------------------------------------------------------


def rotate_point(relative_point, theta_rad, origin):
    """
    Rotate a point from relative coordinates to absolute coordinates using a rotation matrix.

    Parameters:
    relative_point (tuple): A point in relative coordinates (x, y).
    theta (float): Rotation angle in degrees.
    origin (tuple): The origin point about which the rotation is performed.

    Returns:
    tuple: A point in absolute coordinates after rotation.
    """

    # Rotation matrix
    rotation_matrix = np.array([[np.cos(theta_rad), -np.sin(theta_rad)],
                                [np.sin(theta_rad),  np.cos(theta_rad)]])

    # Convert the points to numpy arrays
    relative_point_array = np.array(relative_point)
    origin_array = np.array(origin)

    # Shift the point back to the origin, rotate, and then shift back
    shifted_point = relative_point_array - origin_array
    rotated_point = np.dot(rotation_matrix, shifted_point)
    absolute_point = rotated_point + origin_array

    return absolute_point

# UWB distance data(we used GPS data to find the distance)-------------------------------------------------------------


async def gps_distance(uavs, drone_lat_long, relative_distances, target_coordinate, tracker_coordinate, coordinates):
    lat_long_distance = lat_long_dist()
    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.5)  # 模拟每 0.5 秒获取一次距离
    distance = lat_long_distance.getDistance(drone_lat_long[0].longitude, drone_lat_long[0].latitude, drone_lat_long[0].altitude,
                                             drone_lat_long[1].longitude, drone_lat_long[1].latitude, drone_lat_long[1].altitude)
    print(f"distance: {distance:7.3f}")
    relative_distances.append(distance)
    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await offboard_setup.local_position(uavs[0], tracker_coordinate)
    await offboard_setup.local_position(uavs[1], target_coordinate)
    add_coordinates(coordinates, drone_lat_long)


def add_coordinates(coordinates, drone_lat_long):
    tracker_gps = (drone_lat_long[0].longitude,
                   drone_lat_long[0].latitude, drone_lat_long[0].altitude)
    target_gps = (drone_lat_long[1].longitude,
                  drone_lat_long[1].latitude, drone_lat_long[1].altitude)
    coordinates.append((tracker_gps, target_gps))

# Speed Estimation Model =============================================================================================


def speed_estimate_function(distance, time_rate):
    velocity = ((distance[-1]**2+distance[-3]**2-2 *
                distance[-2]**2)/2*time_rate**2)**0.5
    return velocity


async def calculate_initial_velocity(drone_lat_long):
    relative_distances = []
    number_of_measurements = 3
    measurement_interval = 0.5  # seconds

    for _ in range(number_of_measurements):
        distance = lat_long_distance.getDistance(drone_lat_long[0].longitude, drone_lat_long[0].latitude, drone_lat_long[0].altitude,
                                                 drone_lat_long[1].longitude, drone_lat_long[1].latitude, drone_lat_long[1].altitude)
        print(f"speed estimation distance: {distance:7.3f}")
        relative_distances.append(distance)
        await asyncio.sleep(measurement_interval)

    initial_velocity = speed_estimate_function(
        relative_distances, measurement_interval)
    initial_velocity = initial_velocity + \
        1.0 if initial_velocity > 1.0 else initial_velocity * 10
    initial_velocity = max(1, min(3, initial_velocity))
    print(f"initial_velocity: {initial_velocity:7.3f}")
    return initial_velocity

# local position by IMU+KF =========================================================================================


class SimpleKalmanFilter:
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0  # 可以调整这个值

    def update(self, measurement):
        # 更新过程（可能需要调整方差值）
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / \
            (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + \
            blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (
            1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

# -----------------------------------------------------------------------------------------------
# 2024.02.23 UWB distance data


async def distance_data(relative_distances):
    distance = await uwb_read.UWB_distance()
    print(f"distance: {distance:7.3f}")
    relative_distances.append(distance)
