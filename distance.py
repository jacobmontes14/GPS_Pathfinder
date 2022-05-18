import math


class Distance:
    LATITUDE = 0
    LONGITUDE = 1
    TO_RADIANS = math.pi/180
    RADIUS = 6371e3


    def __init__(self):
        None

    def get_distance(self, location_a, location_b):
        start = self.to_radians(location_a)
        dest = self.to_radians(location_b)

        phi1 = start[self.LATITUDE]
        phi2 = dest[self.LATITUDE]

        delta_phi = self.get_difference(phi2, phi1)
        delta_lam = self.get_difference(dest[self.LONGITUDE], start[self.LONGITUDE])

        a = self.get_haversine(phi1, phi2, delta_phi, delta_lam)
        c = self.get_c(a)

        return self.RADIUS * c


    def get_haversine(self, phi1, phi2, delta_phi, delta_lam):
        delta_phi_sin2 = math.sin(delta_phi/2)**2
        phi_cos = math.cos(phi1) * math.cos(phi2)
        delta_lam_sin2 = math.sin(delta_lam/2)**2

        return delta_phi_sin2 + phi_cos * delta_lam_sin2

    def get_c(self, haversine):
        return 2 * math.atan2(math.sqrt(haversine), math.sqrt(1-haversine))

    def get_difference(self,x, y):
        return x - y

    def to_radians(self, location):
        location[0] = math.radians(location[0])
        location[1] = math.radians(location[1])
        return location