import rospy
import numpy as np
import math
import lat_long_dist
import csv

# initail guess-----------------------------------------------------------------------------------------------------


def z_guess(distance1, distance2, move_rate, z_initial):
    z_guess = z_initial + (distance1**2-distance2**2 +
                           (move_rate)**2)/(2*(move_rate))
    print(f"z initail: {z_initial}")
    print(f"target z position: {z_guess}")
    return z_guess

# 利用固定點迭代, 兩次距離計算目標位置


def fun(distance1, distance2, move_rate, angle):
    up = (distance2**2 - distance1**2 + move_rate**2 + 2 * move_rate * distance2 *
          (-angle**3 / math.factorial(3) + angle**5 / math.factorial(5) - angle**7 / math.factorial(7)))
    down = -2 * move_rate * distance2
    return up / down


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
    # dLR=math.sqrt((Lpre_abs_target[0]-R_abs_target[0])**2+(Lpre_abs_target[1]-R_abs_target[1])**2)
    dLL = math.sqrt((Lpre_abs_target[0]-L_abs_target[0])
                    ** 2+(Lpre_abs_target[1]-L_abs_target[1])**2)
    # dRL=math.sqrt((Rpre_abs_target[0]-L_abs_target[0])**2+(Rpre_abs_target[1]-L_abs_target[1])**2)

    dLR = 10
    dRL = 10
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


def add_coordinates(uavs, coordinates):
    """
    Append current GPS coordinates of drones to the coordinates list.
    """
    tracker_gps = uavs[0].get_current_pose()
    if tracker_gps:
        coordinates.append(tracker_gps)
    else:
        print("Valid GPS coordinates not available for appending.")

# data store in csv ================================================================================================


def export_to_csv(filename, data):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(data)


def prepare_data_for_csv(target_coordinate, predict_target_coordinate, tracker_coordinate, relative_distances):
    # Assuming all lists are of the same length
    combined_data = zip(target_coordinate, predict_target_coordinate,
                        tracker_coordinate, relative_distances)
    formatted_data = [(tc[0], tc[1], tc[2], ptc[0], ptc[1], ptc[2],
                       trc[0], trc[1], trc[2], rd) for tc, ptc, trc, rd in combined_data]
    return formatted_data
