#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (
    OffboardError, VelocityNedYaw, PositionNedYaw, VelocityBodyYawspeed)
import time
from drone_info import Drone_Info


async def drones_connect(drones):
    for num, drone in enumerate(drones):
        await drone.connect(system_address="serial:///dev/ttyUSB0:921600")

        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Drone {num+1} discovered!")
                break
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position and home position ok")
                break


async def drones_arm(drones):
    for drone in drones:
        await drone.action.arm()
        await asyncio.sleep(0.01)


async def start_drones(drones, boids, height):
    try:
        """Connect mavsdk_server to udp port"""

        print("Waiting for drone to connect...")
        await drones_connect(drones)

        print("-- Arming")
        await drones_arm(drones)

        print("-- Taking off")
        await drones_takeoff(drones, height)
        await asyncio.sleep(10)

        print("initial boids")
        await initial_boids(drones, boids)
        print("Start long&lat")
        await get_drone_long_lat(drones, boids)
        await get_drone_local_pos(drones, boids)

        print("-- Starting offboard")
        await start_offboard_mode(drones)
        for drone in drones:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        print("-- Setting initial setpoint")

    except:
        await set_drone_velocity(drones, 0.0, 0.0, 0.0)
        await asyncio.sleep(1)
        await stop_offboard_mode(drones)
        await drones_return_and_land(drones)


async def drones_takeoff(drones, height):

    for drone in drones:
        await drone.action.set_takeoff_altitude(height)
        await drone.action.takeoff()
        await asyncio.sleep(0.01)


async def drones_return_and_land(drones):
    for drone in drones:
        await drone.action.return_to_launch()


# offboard mode
async def start_offboard_mode(drones):
    for drone in drones:
        try:
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            await drone.offboard.start()
        except OffboardError as error:
            print(
                f"Starting offboard mode failed with error code:{error._result.result}")
            exit()


async def get_drone_long_lat(drones, boids):
    for num, drone in enumerate(drones):
        async for position in drone.telemetry.position():
            boids[num].latitude = position.latitude_deg
            boids[num].longitude = position.longitude_deg
            boids[num].altitude = position.relative_altitude_m
            await asyncio.sleep(0.01)
            break


async def get_drone_local_pos(drones, boids):
    for num, drone in enumerate(drones):
        async for position in drone.telemetry.position_velocity_ned():
            boids[num].local_x = position.position.east_m
            boids[num].local_y = position.position.north_m
            boids[num].local_z = (-1)*position.position.down_m
            await asyncio.sleep(0.01)
            break

# 获取并打印相对于起飞点的局部 XYZ 坐标


async def local_position(drone, coordinate):
    async for position in drone.telemetry.position_velocity_ned():
        local_x = position.position.east_m
        local_y = position.position.north_m
        local_z = (-1)*position.position.down_m  # 转换为相对高度
        await asyncio.sleep(0.01)
        coordinate.append([local_x, local_y, local_z])
        break


async def set_drone_velocity(drone, velocity_N, velocity_E, velocity_D):
    await drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_N, velocity_E, velocity_D, 0.0))


async def set_drone_circle(drone):
    print("-- Fly a circle")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(1.0, 0.0, 0.0, 60.0))
    await asyncio.sleep(30)


async def stop_offboard_mode(drones):
    for drone in drones:
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(
                f"Stopping offboard mode failed with error code:{error._result.result}")


async def drones_land(drones):
    print("-- Landing")
    for drone in drones:
        await drone.action.land()

# boids


async def initial_boids(drones, boids):
    for num, drone in enumerate(drones):
        async for position in drone.telemetry.position():
            boids.append(Drone_Info(position.longitude_deg,
                         position.latitude_deg, position.relative_altitude_m))
            break


async def doing_boids(drones, boids, start_time):
    while True:
        for num, drone in enumerate(drones):
            async for position in drone.telemetry.position():
                boids[num].latitude = position.latitude_deg
                boids[num].longitude = position.longitude_deg
                await asyncio.sleep(0.01)
                break

        for num, drone in enumerate(drones):
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(float(boids[num].dy), float(boids[num].dx), 0.0, 0.0))

        print(f"{time.time() - start_time:5.3f}")
        if (time.time() - start_time > 90):
            print("-- Return & Land")
            await stop_offboard_mode(drones)
            await drones_return_and_land(drones)
            break


async def move_position(drone, target_north, target_east, target_down):
    target_position = PositionNedYaw(
        target_north, target_east, target_down, 0.0)
    await drone.offboard.set_position_ned(target_position)
    await asyncio.sleep(1)
