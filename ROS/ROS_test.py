#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import threading

# 全局變量來存儲當前狀態
current_state = State()

def state_cb(state):
    global current_state
    current_state = state

if __name__ == "__main__":
    rospy.init_node('uav_control_node', anonymous=True)
    rate = rospy.Rate(20.0)  # ROS rate 20Hz

    # 訂閱無人機的狀態
    rospy.Subscriber("/mavros/state", State, state_cb)

    # 等待MAVROS服務
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')

    # 服務代理
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # 等待無人機連接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # 設定目標高度
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # 發布目標位置
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # 等待FCU連接
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # 設定OFFBOARD模式並解鎖
    set_mode_client(custom_mode="OFFBOARD")
    arming_client(True)

    # 主循環
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        rate.sleep()

        # 檢查是否達到目標高度，這裡僅為示例，實際應用中需要更精確的條件
        if pose.pose.position.z - 0.1 < current_state.pose.position.z < pose.pose.position.z + 0.1:
            print("達到目標高度，準備降落")
            break

    # 設定模式為AUTO.LAND並降落
    set_mode_client(custom_mode="AUTO.LAND")
