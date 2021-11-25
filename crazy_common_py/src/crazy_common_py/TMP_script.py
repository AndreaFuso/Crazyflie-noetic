from math import atan2, sqrt, sin, cos, pi

def computeSpeeds(coord_list, w):
    velocity_list = []
    cont = 1
    for ii in range(0, len(coord_list)):
        psi_0 = atan2(coord_list[ii][1], coord_list[ii][0])
        r = sqrt(coord_list[ii][0] ** 2 + coord_list[ii][1] ** 2)
        vx = - w * r * sin(psi_0)
        vy = w * r * cos(psi_0)
        velocity_list.append([vx, vy])
        print(f'CF{cont}; Vx = {vx}; Vy = {vy}')
        cont += 1
    return velocity_list


w = 45 * pi / 180

coord = [[0, 0],
         [-0.5, -0.5],
         [-0.5, 0.5],
         [0.5, 0.5],
         [0.5, -0.5],
         [-1, -1],
         [-1, 0],
         [-1, 1],
         [0, 1],
         [1, 1],
         [1, 0],
         [1, -1],
         [0, -1]]

computeSpeeds(coord, w)