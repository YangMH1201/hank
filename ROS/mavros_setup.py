import rospy
import threading
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


class DroneController:
    def __init__(self, drone_name):
        self.namespace = f"/{drone_name}"
        self.current_state = State()
        self.current_extended_state = ExtendedState()
        self.current_global_position = NavSatFix()
        self.current_local_position = None  # 存储最新的本地位置数据
        self.connection_reported = False
        self.position_reported = False

        # Subscribers
        rospy.Subscriber(f"{self.namespace}/mavros/state",
                         State, self.state_cb)
        rospy.Subscriber(f"{self.namespace}/mavros/extended_state",
                         ExtendedState, self.extended_state_cb)
        rospy.Subscriber(f"{self.namespace}/mavros/global_position/global",
                         NavSatFix, self.global_position_cb)
        rospy.Subscriber(
            f"{self.namespace}/mavros/local_position/pose", PoseStamped, self.pose_callback)

        # Service Clients
        self.arming_client = rospy.ServiceProxy(
            f"{self.namespace}/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(
            f"{self.namespace}/mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy(
            f"{self.namespace}/mavros/cmd/takeoff", CommandTOL)
        self.land_client = rospy.ServiceProxy(
            f"{self.namespace}/mavros/cmd/land", CommandTOL)
        self.vel_pub = rospy.Publisher(
            f"{self.namespace}/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

    def state_cb(self, state_msg):
        self.current_state = state_msg
        if self.current_state.connected and not self.connection_reported:
            rospy.loginfo(f"{self.namespace} - Drone connected!")
            self.connection_reported = True

    def extended_state_cb(self, extended_state_msg):
        self.current_extended_state = extended_state_msg

    def global_position_cb(self, global_position_msg):
        self.current_global_position = global_position_msg
        if self.current_global_position.status.status == 0 and not self.position_reported:  # Checks if fix is valid
            if self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                rospy.loginfo(
                    f"{self.namespace} - Global position and home position ok")
                self.position_reported = True

    def pose_callback(self, pose_msg):
        """Store the current pose data."""
        self.current_pose = pose_msg.pose

    def get_current_pose(self):
        """Return the current pose, if available."""
        if self.current_pose:
            return (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
        else:
            return None

    def arm(self):
        return self.arming_client(True)

    def disarm(self):
        return self.arming_client(False)

    def set_mode(self, mode):
        return self.set_mode_client(custom_mode=mode)

    def takeoff(self, altitude):
        if self.current_global_position.latitude == 0.0 and self.current_global_position.longitude == 0.0:
            rospy.logerr("No valid GPS fix available.")
            return False
        else:
            return self.takeoff_client(altitude=altitude, latitude=self.current_global_position.latitude, longitude=self.current_global_position.longitude, min_pitch=0, yaw=0)

    def land(self):
        return self.land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)

    def set_velocity(self, linear_x, linear_y, linear_z, duration, frequency=20):
        rate = rospy.Rate(frequency)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_x
        velocity_msg.linear.y = linear_y
        velocity_msg.linear.z = linear_z

        try:
            while rospy.Time.now() < end_time:
                self.vel_pub.publish(velocity_msg)
                rate.sleep()
        finally:
            # 確保無人機停止
            velocity_msg.linear.x = 0
            velocity_msg.linear.y = 0
            velocity_msg.linear.z = 0
            self.vel_pub.publish(velocity_msg)

    def get_gps_coordinates(self):
        """Returns the current GPS coordinates and altitude of the drone."""
        if self.current_global_position.latitude == 0.0 and self.current_global_position.longitude == 0.0:
            rospy.logwarn(f"{self.namespace} - No valid GPS fix available.")
            return None
        else:
            return (
                self.current_global_position.latitude,
                self.current_global_position.longitude,
                self.current_global_position.altitude
            )

    def continuous_flight(self, speedx, speedy, speedz):
        """使无人机持续前进。"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.set_velocity(speedx, speedy, speedz)  # 只在x轴方向上设置速度，以实现向前飞行
            rate.sleep()


def setup_drones():
    drone1 = DroneController("uav0")
    drone2 = DroneController("uav1")
    return [drone1, drone2]


def land_drones(drones):
    for drone in drones:
        drone.land()
        drone.disarm()


def maintain_offboard_control(drone):
    rate = rospy.Rate(20)  # 设置发送频率为 20Hz
    vel_msg = Twist()  # 创建一个零速度的Twist消息
    while not rospy.is_shutdown() and drone.current_state.mode != "OFFBOARD":
        drone.vel_pub.publish(vel_msg)  # 持续发送零速度指令
        rate.sleep()


def attempt_set_mode_and_arm(drone, attempts=3):
    for attempt in range(attempts):
        rospy.loginfo(
            f"Attempt {attempt + 1}: Setting mode to OFFBOARD for {drone.namespace}")
        response = drone.set_mode("OFFBOARD")
        if response.mode_sent:
            rospy.loginfo(f"-- Mode set to OFFBOARD for {drone.namespace}")
            rospy.sleep(0.5)
            rospy.loginfo(f"Attempt {attempt + 1}: Arming {drone.namespace}")
            response = drone.arm()
            if response.success:
                rospy.loginfo(f"-- {drone.namespace} is armed")
                return True
            else:
                rospy.logerr(f"-- Failed to arm {drone.namespace}")
        else:
            rospy.logerr(
                f"-- Failed to set mode to OFFBOARD for {drone.namespace}")
        rospy.sleep(1)  # Sleep before the next attempt
    return False


def hover(drone):
    """Maintains the current position of the drone by publishing zero velocities."""
    rospy.loginfo(f"{drone.namespace} is now hovering.")
    rate = rospy.Rate(10)  # 10 Hz
    zero_vel = Twist()
    while not rospy.is_shutdown():
        # Send zero velocity to maintain position
        drone.set_velocity(0, 0, 0, 0)
        rate.sleep()


def start_drones(drones, heights):
    threads = []
    # 同步條件變量
    hover_condition = threading.Condition()
    # 懸停狀態追蹤
    hover_status = {drone.namespace: False for drone in drones}

    def set_hover_status(namespace, status):
        nonlocal hover_status
        with hover_condition:
            hover_status[namespace] = status
            # 檢查所有無人機是否都已懸停
            all_hovered = all(hover_status.values())
            if all_hovered:
                # 如果都已懸停，通知等待的線程
                hover_condition.notify_all()

    for drone in drones:
        rospy.loginfo(f"-- Waiting for {drone.namespace} to connect")
        while not drone.current_state.connected:
            rospy.sleep(0.5)

        # Start continuous control command
        thread = threading.Thread(
            target=maintain_offboard_control, args=(drone,))
        thread.start()
        threads.append(thread)

        if attempt_set_mode_and_arm(drone):
            # 使用 get 方法來獲取高度，如果命名空間不存在則返回默認值 3
            desired_height = heights.get(drone.namespace, 3)
            rospy.loginfo(
                f"-- Taking off {drone.namespace} to {desired_height} meters")
            success = drone.takeoff(desired_height)
            if success:
                rospy.loginfo(
                    f"Takeoff initiated for {drone.namespace}, waiting for stabilization.")
                rospy.sleep(10)
                rospy.loginfo(
                    f"{drone.namespace} has likely reached its altitude and stabilized.")
                hover_thread = threading.Thread(target=hover, args=(drone,))
                hover_thread.start()
                threads.append(hover_thread)
                set_hover_status(drone.namespace, True)
            else:
                rospy.logerr(
                    f"Takeoff failed for {drone.namespace}, check GPS fix and try again.")
        else:
            rospy.logerr(
                f"Failed to set mode to OFFBOARD and arm {drone.namespace} after multiple attempts")

    with hover_condition:
        while not all(hover_status.values()):
            hover_condition.wait()

    rospy.loginfo(
        "All drones are hovered and stable, proceeding to the next step.")
