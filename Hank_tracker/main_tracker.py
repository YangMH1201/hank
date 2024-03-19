import asyncio
from mavsdk import System
import offboard_setup
import find_coordinate


async def main():

    # initialize paramters
    tracker_height = 5.0

    """ follower/target system initialization """
    uav_tracker = System()
    uavs = [uav_tracker]
    drone_lat_long = list()

    """ Start Algo initailization """
    await offboard_setup.start_drones(uavs, drone_lat_long, tracker_height)
    await asyncio.sleep(1)

    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await offboard_setup.get_drone_local_pos(uavs, drone_lat_long)

    await offboard_setup.get_drone_long_lat(uavs, drone_lat_long)
    await asyncio.sleep(0.1)

    # Algorithm to update the target
    await find_coordinate.start_mission(uavs, drone_lat_long)

    # close to the target
    print("Mission Complete!")
    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
    await asyncio.sleep(5)
    await offboard_setup.stop_offboard_mode(uavs[0])

    print("Landing...")
    await offboard_setup.drones_land(uavs[0])

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(main()))
