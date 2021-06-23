import math

def constrain(value, lowerVal, upperVal):
    if value < lowerVal:
        return lowerVal
    if value > upperVal:
        return upperVal
    return value

def quat2euler(x, y, z, w):
    # Roll:
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch:
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw:
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)

def rad2deg(rad):
    return (rad * 180.0 / math.pi)

def deg2rad(deg):
    return (deg * math.pi / 180.0)