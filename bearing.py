import math
from operator import mod

class Bearing:
    LATITUDE = 0
    LONGITUDE = 1

    def __init__(self):
        None

    def get_x(self, start, dest):
        return (math.cos(start[self.LATITUDE]) * math.sin(dest[self.LATITUDE])) - (math.sin(start[self.LATITUDE]) * math.cos(dest[self.LATITUDE]) * math.cos(self.get_difference(dest[self.LONGITUDE], start[self.LONGITUDE])))

    def get_y(self, start, dest):
        return math.sin(self.get_difference(dest[self.LONGITUDE], start[self.LONGITUDE]) * math.cos(dest[self.LATITUDE]))

    def get_bearing(self, y, x):
        bearing = math.atan2(y, x)
        bearing_in_degrees = (((bearing*180) / (math.pi)) + 360) % 360
        return bearing_in_degrees

    def to_radians(self, location):
        location[0] = math.radians(location[0])
        location[1] = math.radians(location[1])
        return location

    def get_difference(self,x, y):
        return x - y

    def find_bearing(self, location_a, location_b):
        start_pos = self.to_radians(location_a)
        dest = self.to_radians(location_b)

        y = self.get_y(start_pos, dest)
        x = self.get_x(start_pos, dest)

        return self.get_bearing(y, x)

    def find_bearing_test_2(self, location_a, location_b):
        start_pos = self.to_radians(location_a)
        dest = self.to_radians(location_b)

        val_1 = (dest[self.LATITUDE]/2) + (math.pi/4)
        val_2 = math.tan((start_pos[self.LATITUDE]/2) + (math.pi) /4)

        phi = math.log(math.tan(val_1 / val_2))
        lon = abs(start_pos[self.LONGITUDE] - dest[self.LONGITUDE])

        if(lon > 180):
            lon = lon*mod(180)

        bearing = math.atan2(lon, phi)
        return math.degrees(bearing)

