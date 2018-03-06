import numpy as np
import math

#match xy axes by rotatin anti - clock by angle theta
def rot_k(theta):
	theta = np.radians(theta)
	c, s = np.cos(theta), np.sin(theta)
	R = np.matrix([[c, s, 0],
				   [-s, c, 0],
				   [0, 0, 1]])
	return R

#match xz axes by rotatin anti - clock by angle theta
def rot_j(theta):
	theta = np.radians(theta)
	c, s = np.cos(theta), np.sin(theta)
	R = np.matrix([[c, 0, -s],
				   [0, 1, 0],
				   [s, 0, c]])
	return R

#match yz axes by rotatin anti - clock by angle theta
def rot_i(theta):
	theta = np.radians(theta)
	c, s = np.cos(theta), np.sin(theta)
	R = np.matrix([[1, 0, 0],
				   [0, c, s],
				   [0, -s, c]])
	return R

def LLHtoECEF(lat, lon, alt):
    # see http://www.mathworks.de/help/toolbox/aeroblks/llatoecefposition.html

    rad = np.float64(6378137.0)        # Radius of the Earth (in meters)
    f = np.float64(1.0/298.257223563)  # Flattening factor WGS84 Model
    lat = np.radians(lat)
    lon = np.radians(lon)
    cosLat = np.cos(lat)
    sinLat = np.sin(lat)
    FF     = (1.0-f)**2
    C      = 1/np.sqrt(cosLat**2 + FF * sinLat**2)
    S      = C * FF

    x = (rad * C + alt)*cosLat * np.cos(lon)
    y = (rad * C + alt)*cosLat * np.sin(lon)
    z = (rad * S + alt)*sinLat

    return np.matrix([x, y, z])


# a = rot_k(30)
# b = rot_j(30)
# c = rot_i(30)
# print(a)
# print(b)
# print(c)

# phi - vehicle 1 frame in degrees
plane_azimuth = 30

# theta - vehicle 2 frame in degrees
plane_pitch = 45

# roll - vehicle 3 frame in degrees
plane_roll = 60

#latitude of plane in degrees
plane_lat = 34.0522

#longitude of plane in degrees
plane_lon = -118.40806

#altitude of plane in metres
plane_alt = 0

#latitude of plane in degrees
object_lat = 20

#longitude of plane in degrees
object_lon = 120

#altitude of plane in metres
object_alt = 6


# Inertial to body framem v : inertial frame, b: body frame
R_v_v1 = rot_k(plane_azimuth)
R_v1_v2 = rot_j(plane_pitch)
R_v2_b = rot_i(plane_roll)

R_v_v2 = np.dot(R_v1_v2, R_v_v1) 
R_v_b = np.dot(R_v2_b, R_v_v2)

# print(R_v_b)
# print(LLHtoECEF(34.0522, -118.40806, 0))

#Converting plane LLH to ECEF
planeECEF = LLHtoECEF(plane_lat, plane_lon, plane_alt)

#Converting Object LLH to ECEF
objectECEF = LLHtoECEF(object_lat, object_lon, object_alt)

# print(planeECEF)
# print(objectECEF)
l_d_i = objectECEF - planeECEF
l_d_i_unit = np.transpose((1.0 / np.linalg.norm(l_d_i)) * l_d_i)
#print(l_d_i)
#print(l_d_i_unit)

l_d_b = np.dot(R_v_b, l_d_i_unit)
print("Desired unit vector")
print(l_d_b)

desired_azimuth = math.degrees(math.atan(l_d_b[1][0] / l_d_b[0][0]))
desired_elevation = math.degrees(math.asin(l_d_b[2][0]))

print("Desired desired_azimuth (degree)")
print(desired_azimuth)

print("Desired desired_elevation (degree)")
print(desired_elevation)