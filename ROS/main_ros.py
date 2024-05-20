import rospy
import mavros_setup
import find_coordinate
import function
import threading
from geometry_msgs.msg import Twist
import time
from uwb_read import UwbModule


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


def main():
    uwb_info = UwbModule()
    # initialize paramters
    rospy.init_node('drone_controller')

    # Initialize the list for keeping track of positions and velocities
    relative_distances, drone_lat_long = [], []
    target_coordinate, tracker_coordinate, prd_target_coordinate = [], [], []

    # Initialize drones
    drones = mavros_setup.setup_drones()
    heights = 3.0
    # Start drones
    mavros_setup.start_drones(drones, heights)

    """ uwb status check """
    uwb_dist = get_uwb_dist(uwb_info)
    if uwb_dist == 9999:
        drones[0].set_velocity(0.0, 0.0, 0.0, duration=1)
        rospy.sleep()
        # await offboard_setup.stop_offboard_mode(uavs)
        drones[0].land()
        raise
    print(f"Initial uwb dist = {uwb_dist}")
    print("UWB info stable...")

    # Algorithm to update the target
    find_coordinate.start_mission(drones, drone_lat_long, tracker_coordinate,
                                  target_coordinate, prd_target_coordinate, relative_distances, uwb_info)

    print("Mission Complete!")

    # Landing
    print("Landing...")
    drones[0].land()

    # Export data to CSV
    formatted_data = function.prepare_data_for_csv(
        target_coordinate, prd_target_coordinate, tracker_coordinate, relative_distances)
    function.export_to_csv('output.csv', formatted_data)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
