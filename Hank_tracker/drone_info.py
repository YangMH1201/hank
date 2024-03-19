#!/usr/bin/env python3

import math
import numpy as np
from lat_long_dist import lat_long_dist

class Coordinate:
    def __init__(self, longitude, latitude, altitude):
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude

class VelocityChange:
    def __init__(self, dx, dy, dz):
        self.dx = dx
        self.dy = dy
        self.dz = dz
        
class Drone_Info:
    def __init__(self, longitude, latitude, altitude):
        self.prev_local_x = 0.0
        self.prev_local_y = 0.0
        self.prev_local_z = 0.0
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        
    def distance(self, drone_info):
        return self.__distance_function.getDistance(self.longitude, self.latitude, self.altitude, drone_info.longitude, drone_info.latitude, drone_info.altitude)
    