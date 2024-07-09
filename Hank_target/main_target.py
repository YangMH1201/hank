#!/usr/bin/env python
import asyncio
from mavsdk import System
import offboard_setup
from uwb_read import UwbModule
import time


async def main():
    uwb_info = UwbModule()

    # initialize paramters
    target_height = 3.0
    approach_distance = 2.5
    velocity_North = 0.1
    velocity_East = 0.0
    velocity_down = 0.0

    number = 0
    drone_lat_long = list()

    # Connect to the Simulation
    uav_target = System()
    uavs = [uav_target]

    try:
        """ Start Algo initailization """
        await offboard_setup.start_drones(uavs, drone_lat_long, target_height)
        print("Start Algo")
        asyncio.ensure_future(offboard_setup.set_drone_velocity(
            uavs[0], velocity_North, velocity_East, velocity_down))

        uwb_err = 0
        """UWB data check"""
        uwb_check = uwb_info.get_module_data()
        if uwb_check == None:
            while uwb_check == None:
                if uwb_err >= 80:
                    print("UWB data err........")
                    await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
                    await asyncio.sleep(5)
                    await offboard_setup.stop_offboard_mode(uavs)
                    await uavs[0].action.land()
                    raise
                uwb_check = uwb_info.get_module_data()
                uwb_err += 1
                time.sleep(0.05)

        uwb_err = 0
        uwb_check = float(uwb_check)
        print(f"Initial uwb dist = {uwb_check}")
        print("UWB info stable...")

        """Avoidance range check"""
        uwb_dist = uwb_check
        while uwb_dist > 2.3 or uwb_dist < 17:
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

            uwb_err = 0
            uwb_dist = float(uwb_dist)
            print(f"current dist : {uwb_dist}")
            time.sleep(0.1)

            if uwb_dist < approach_distance:
                number += 1
                if number >= 5:
                    break
            else:
                number = 0
                await asyncio.sleep(0.1)
    except:
        await offboard_setup.set_drone_velocity(uavs[0], 0.0, 0.0, 0.0)
        await asyncio.sleep(5)
        await offboard_setup.stop_offboard_mode(uavs)
        await uavs[0].action.land()
        raise

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
