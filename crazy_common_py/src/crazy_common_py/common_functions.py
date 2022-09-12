import math
from crazy_common_py.dataTypes import Vector3
def constrain(value, lowerVal, upperVal):
    if value < lowerVal:
        return lowerVal
    elif value > upperVal:
        return upperVal
    else:
        return value

def quat2euler(x, y, z, w):
    ''''# Roll:
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
    yaw = math.atan2(siny_cosp, cosy_cosp)'''
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)

def rad2deg(rad):
    return (rad * 180.0 / math.pi)

def deg2rad(deg):
    return (deg * math.pi / 180.0)

def sameSign(number1, number2):
    if (number1 > 0 and number2 > 0) or (number1 < 0 and number2 < 0 ):
        return True
    else:
        return False

def isSameVector(vector1 = Vector3(), vector2 = Vector3()):
    if vector1.x == vector2.x and vector1.y == vector2.y and vector1.z == vector2.z:
        return True
    else:
        return False


def RotateVector(vector=Vector3(), rotation=Vector3()):
    cosR = math.cos(rotation.x)
    sinR = math.sin(rotation.x)

    cosP = math.cos(rotation.y)
    sinP = math.sin(rotation.y)

    cosY = math.cos(rotation.z)
    sinY = math.sin(rotation.z)

    # rotation matrix
    R11 = cosY * cosP
    R12 = cosY * sinP * sinR - sinY * cosR
    R13 = cosY * sinP * cosR + sinY * sinR
    R21 = sinY * cosP
    R22 = sinY * sinP * sinR + cosY * cosR
    R23 = sinY * sinP * cosR - cosY * sinR
    R31 = - sinP
    R32 = cosP * sinR
    R33 = cosP * cosR

    output = Vector3()

    output.x = vector.x * R11 + vector.y * R12 + vector.z * R13
    output.y = vector.x * R21 + vector.y * R22 + vector.z * R23
    output.z = vector.x * R31 + vector.y * R32 + vector.z * R33

    return output

def standardNameList(number_of_cfs):
    cf_names = []
    for number in range(1, number_of_cfs + 1):
        cf_names.append('cf' + str(number))
    return cf_names

def extractCfNumber(name):
    # print('name is: ', name)
    digit_pos = [name.find('1'), name.find('2'), name.find('3'), name.find('4'), name.find('5'), name.find('6'),
                 name.find('7'), name.find('8'), name.find('9')]
    for ii in range(0, len(digit_pos)):
        if digit_pos[ii] == -1:
            digit_pos[ii] = 100

    first_digit_pos = min(digit_pos)
    # print('name[first_digit_pos:] is: ', name[first_digit_pos:])
    return int(name[first_digit_pos:])