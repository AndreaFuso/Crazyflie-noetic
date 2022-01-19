import numpy as np
from crazy_common_py.dataTypes import Vector3

class TextFormation:
    def __init__(self, char_height, char_width, density, spawning_altitude):
        self.char_height = char_height
        self.char_width = char_width
        self.density = density
        self.spawning_altitude = spawning_altitude
        self.first_char = True
        self.space_width = char_width + char_width / 2

    def getCoords(self, word):
        initial_position = 0.0
        for ii in range(0, len(word)):
            if word[ii] == ' ':
                initial_position = initial_position + self.space_width * 2
            if word[ii] == 'A':
                if self.first_char:
                    self.coords = self.__generate_coord_A(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_A = self.__generate_coord_A(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_A))
            if word[ii] == 'H':
                if self.first_char:
                    self.coords = self.__generate_coord_H(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_H = self.__generate_coord_H(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_H))
            if word[ii] == 'N':
                if self.first_char:
                    self.coords = self.__generate_coord_N(initial_pos=Vector3(), des_char_height=2,
                                                          des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_N = self.__generate_coord_N(initial_pos=Vector3(initial_position, 0, 0),
                                                      des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_N))
            if word[ii] == 'K':
                if self.first_char:
                    self.coords = self.__generate_coord_K(initial_pos=Vector3(), des_char_height=2,
                                                          des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_K = self.__generate_coord_K(initial_pos=Vector3(initial_position, 0, 0),
                                                      des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_K))
            if word[ii] == 'T':
                if self.first_char:
                    self.coords = self.__generate_coord_T(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_T = self.__generate_coord_T(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_T))
            if word[ii] == 'Y':
                if self.first_char:
                    self.coords = self.__generate_coord_Y(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_Y = self.__generate_coord_Y(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_Y))
            if word[ii] == 'O':
                if self.first_char:
                    self.coords = self.__generate_coord_O(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_O = self.__generate_coord_O(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_O))
            if word[ii] == 'U':
                if self.first_char:
                    self.coords = self.__generate_coord_U(initial_pos=Vector3(), des_char_height=2, des_char_width=1)
                    self.first_char = False
                else:
                    initial_position = initial_position + self.space_width
                    coord_U = self.__generate_coord_U(initial_pos=Vector3(initial_position, 0, 0), des_char_height=2,
                                                      des_char_width=1)
                    self.coords = np.concatenate((self.coords, coord_U))
        return self.coords




    def __generate_coord_A(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        first_line = self.__generate_coords_line(start_pos=initial_pos,
                                                 finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                    initial_pos.y + char_height,
                                                                    self.spawning_altitude),
                                                 number_of_points=5)
        # Second line:
        second_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width / 2,
                                                                    initial_pos.y + char_height,
                                                                    self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y,
                                                                    self.spawning_altitude),
                                                 number_of_points=5)
        # Horizontal line:
        horiz_line = self.__generate_coords_line(start_pos=Vector3(first_line[2, 0],
                                                                   first_line[2, 1],
                                                                   self.spawning_altitude),
                                                  finish_pos=Vector3(second_line[2, 0],
                                                                     second_line[2, 1],
                                                                     self.spawning_altitude),
                                                  number_of_points=3)
        # Assembling:
        coordinates_A = np.concatenate((first_line, second_line[1:, :], horiz_line[1:-1, :]))
        return coordinates_A

    def __generate_coord_T(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width / 2,
                                                                      initial_pos.y,
                                                                      self.spawning_altitude),
                                                    finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                       initial_pos.y + char_height,
                                                                       self.spawning_altitude),
                                                    number_of_points=5)
        horiz_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y + char_height,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        # Assembling:
        coordinates_T = np.concatenate((horiz_line, vertical_line[0:-1, :]))
        return coordinates_T

    def __generate_coord_H(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line1 = self.__generate_coords_line(start_pos=initial_pos,
                                                     finish_pos=Vector3(initial_pos.x,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        vertical_line2 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width,
                                                                       initial_pos.y,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x + char_width,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        horiz_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height / 2,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y + char_height / 2,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        # Assembling:
        coordinates_H = np.concatenate((vertical_line1, vertical_line2, horiz_line[1:-1]))
        return coordinates_H

    def __generate_coord_N(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line1 = self.__generate_coords_line(start_pos=initial_pos,
                                                     finish_pos=Vector3(initial_pos.x,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        vertical_line2 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width,
                                                                       initial_pos.y,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x + char_width,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        mid_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y,
                                                                    self.spawning_altitude),
                                                 number_of_points=5)
        # Assembling:
        coordinates_N = np.concatenate((vertical_line1, vertical_line2, mid_line[1:-1]))
        return coordinates_N

    def __generate_coord_K(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line = self.__generate_coords_line(start_pos=initial_pos,
                                                    finish_pos=Vector3(initial_pos.x,
                                                                       initial_pos.y + char_height,
                                                                       self.spawning_altitude),
                                                    number_of_points=5)
        upper_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height / 2,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y + char_height,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        lower_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height / 2,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width,
                                                                    initial_pos.y,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        # Assembling:
        coordinates_K = np.concatenate((vertical_line, upper_line[1:], lower_line[1:]))
        return coordinates_K

    def __generate_coord_Y(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width / 2,
                                                                      initial_pos.y,
                                                                      self.spawning_altitude),
                                                    finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                       initial_pos.y + char_height,
                                                                       self.spawning_altitude),
                                                    number_of_points=3)
        left_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                   initial_pos.y + char_height,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                    initial_pos.y + char_height / 2,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        right_line = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width,
                                                                   initial_pos.y + char_height,
                                                                   self.spawning_altitude),
                                                 finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                    initial_pos.y + char_height / 2,
                                                                    self.spawning_altitude),
                                                 number_of_points=3)
        # Assembling:
        coordinates_Y = np.concatenate((vertical_line, left_line[0:-1], right_line[0:-1]))
        return coordinates_Y

    def __generate_coord_O(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line1 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                       initial_pos.y + (1 / 4) * char_height,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x,
                                                                        initial_pos.y + (3 / 4) * char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=3)
        vertical_line2 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width,
                                                                       initial_pos.y + (1 / 4) * char_height,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x + char_width,
                                                                        initial_pos.y + (3 / 4) * char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=3)
        extrema = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width / 2,
                                                                initial_pos.y,
                                                                self.spawning_altitude),
                                              finish_pos=Vector3(initial_pos.x + char_width / 2,
                                                                 initial_pos.y + char_height,
                                                                 self.spawning_altitude),
                                              number_of_points=2)
        # Assembling:
        coordinates_O = np.concatenate((vertical_line1, vertical_line2, extrema))
        return coordinates_O

    def __generate_coord_U(self, initial_pos = Vector3(), des_char_height = 0, des_char_width = 0, des_density = 0):
        # Chars parameters:
        if des_char_height == 0:
            char_height = self.char_height
        else:
            char_height = des_char_height

        if des_char_width == 0:
            char_width = self.char_width
        else:
            char_width = des_char_width

        if des_density == 0:
            density = self.density
        else:
            density = des_density

        # First line:
        vertical_line1 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                                       initial_pos.y,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        vertical_line2 = self.__generate_coords_line(start_pos=Vector3(initial_pos.x + char_width,
                                                                       initial_pos.y,
                                                                       self.spawning_altitude),
                                                     finish_pos=Vector3(initial_pos.x + char_width,
                                                                        initial_pos.y + char_height,
                                                                        self.spawning_altitude),
                                                     number_of_points=5)
        bottom = self.__generate_coords_line(start_pos=Vector3(initial_pos.x,
                                                               initial_pos.y,
                                                               self.spawning_altitude),
                                             finish_pos=Vector3(initial_pos.x + char_width,
                                                                initial_pos.y,
                                                                self.spawning_altitude),
                                             number_of_points=3)
        # Assembling:
        coordinates_U = np.concatenate((vertical_line1, vertical_line2, bottom[1:-1]))
        return coordinates_U

    def __generate_coords_line(self, start_pos = Vector3(), finish_pos = Vector3(), number_of_points = 2):
        positions = np.zeros((number_of_points, 3))
        # Vertical line:
        if start_pos.x == finish_pos.x:
            positions[:, 0] = np.ones((number_of_points, 1))[:, 0] * start_pos.x
            positions[:, 1] = np.linspace(start_pos.y, finish_pos.y, number_of_points)
            positions[:, 2] = np.ones((number_of_points, 1))[:, 0] * self.spawning_altitude
        else:
            # Non vertical line:
            positions[:, 0] = np.linspace(start_pos.x, finish_pos.x, number_of_points)
            for ii in range(0, number_of_points):
                positions[ii, 1] = start_pos.y + ((finish_pos.y - start_pos.y) / (finish_pos.x - start_pos.x)) * \
                                   (positions[ii, 0] - start_pos.x)
            positions[:, 2] = np.ones((number_of_points, 1))[:, 0] * self.spawning_altitude
        return positions