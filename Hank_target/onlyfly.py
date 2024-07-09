
#!/usr/bin/env python
import asyncio
from mavsdk import System
import offboard_setup
import time


async def main():

    # initialize paramters
    target_height = 2.0
    approach_distance = 2.0
    velocity_North = 0.1
    velocity_East = 0.0
    velocity_down = 0.0

    number = 0
    drone_lat_long = list()

    # Connect to the Simulation
    uav_target = System()
    uavs = [uav_target]

    await offboard_setup.start_drones(uavs, drone_lat_long, target_height)
        print("Start Algo")
        asyncio.ensure_future(offboard_setup.set_drone_velocity(
            uavs[0], velocity_North, velocity_East, velocity_down))
    await asyncio.sleep(5)

    # Stop the mission, and land the drone
    print(f"Reach avoidance range : {uwb_dist}")
    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
    await offboard_setup.stop_offboard_mode(uavs)

    print("landing the target drone.")
    await offboard_setup.drones_land(uavs)

    # Wait for the tasks to finish
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
