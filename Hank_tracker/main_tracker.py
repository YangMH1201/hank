import asyncio
from mavsdk import System
import offboard_setup
import find_coordinate
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

async def main():
    uwb_info = UwbModule()
    # initialize paramters
    tracker_height = 5.0

    """ follower/target system initialization """
    uav_tracker = System()
    uavs = [uav_tracker]
    drone_lat_long = list()
    tracker_coordinate = []
    relative_distances = []
    past_target_coordinates = []  # 存儲過去的target_coordinates

    """ Start Algo initailization """
    await offboard_setup.start_drones(uavs, drone_lat_long, tracker_height)
    await asyncio.sleep(1)

    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await offboard_setup.get_drone_local_pos(uavs, drone_lat_long)
    await asyncio.sleep(0.1)

    """ uwb status check """
    uwb_dist = get_uwb_dist(uwb_info)
    if uwb_dist == 9999:
        await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
        await asyncio.sleep(5)
        await offboard_setup.stop_offboard_mode(uavs)
        await uavs[0].action.land()
        raise
    print(f"Initial uwb dist = {uwb_dist}")
    print("UWB info stable...")
    
    # Algorithm to update the target
    await find_coordinate.start_mission(uavs, drone_lat_long, uwb_info, tracker_coordinate, relative_distances, past_target_coordinates)

    # close to the target
    print("Mission Complete!")
    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
    await asyncio.sleep(1)
    await offboard_setup.stop_offboard_mode(uavs)

    print("Landing...")
    await offboard_setup.drones_land(uavs)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(main()))
