import sim
import sys
import math
import numpy
import threading
import time
import pygame
from pygame.locals import *
from os import listdir
from os.path import isfile, join
from communication import Communication
from vehicleCinematics import VehicleCinematics
from vehicleSensors import VehicleSensors
from algorith_astar import *

#path = astar(level, i.position, (tmp_y, tmp_x))
# to del
import mapViewer

maze_map = numpy.zeros((10, 10), dtype=int)


def segmentInserter(maze_map_big, segment, i, j):
    i *= 3
    j *= 3
    k = 0
    l = 0
    print(segment)
    for u in range(i, i+3):
        for y in range(j, j+3):
            maze_map_big[u][y] = segment[l][k]
            # print(maze_map_big[i][j])
            k += 1
        k = 0
        l += 1


def openConverterTemplateFile():
    maze_map = numpy.loadtxt('file.txt', delimiter=",", dtype=int)
    converterSegment = []
    converterTemplate = numpy.loadtxt(
        'converter.txt', delimiter=",", dtype=int)
    rows = converterTemplate.shape[0]
    cols = converterTemplate.shape[1]
    tmp = []
    tmp2 = []
    for i in range(0, rows):
        for j in range(0, cols):
            tmp.append(converterTemplate[i][j])
        tmp2.append(tmp)
        tmp = []
        if ((i + 1) % 3 == 0):
            # print(tmp2)
            converterSegment.append(tmp2)
            tmp2 = []

    # print(converterSegment)
    rows = maze_map.shape[0]
    cols = maze_map.shape[1]
    map_after_convert = numpy.zeros((30, 30), dtype=int)
    for i in range(0, rows):
        for j in range(0, cols):
            # print(maze_map[i][j])
            if maze_map[i][j] != 0:
                segmentInserter(map_after_convert,
                                converterSegment[maze_map[i][j]], i, j)
    print(map_after_convert)
    path = astar(map_after_convert, (28, 1), (28, 4))
    print(path)
    mapViewer.draw_window()


class Map():
    def __init__(self):
        self.interchange = maze_map
        #self.interchange = numpy.zeros((12, 12))
        #self.interchange = self.interchange.astype(int)
        self.interchange[0][0] = 12
        self.old_coord = (0, 0)

    def addStraightSegment(self, x, y, direction):
        if direction == "up" or direction == "down":
            self.interchange[x][y] = 5
        else:
            self.interchange[x][y] = 4
        numpy.savetxt('file.txt', numpy.flipud(
            self.interchange), fmt='%i', delimiter=',')

    def addCornerSegmentRight(self, x, y, direction):
        if direction == "up":
            self.interchange[x][y] = 7
        elif direction == "right":
            self.interchange[x][y] = 6
        elif direction == "down":
            self.interchange[x][y] = 8
        elif direction == "left":
            self.interchange[x][y] = 9

    def addCornerSegmentLeft(self, x, y, direction):
        if direction == "up":
            self.interchange[x][y] = 6
        elif direction == "right":
            self.interchange[x][y] = 8
        elif direction == "down":
            self.interchange[x][y] = 9
        elif direction == "left":
            self.interchange[x][y] = 7

    def addDeadEndSegment(self, x, y, direction):
        if direction == "up":
            self.interchange[x][y] = 10
        elif direction == "right":
            self.interchange[x][y] = 11
        elif direction == "down":
            self.interchange[x][y] = 12
        elif direction == "left":
            self.interchange[x][y] = 13


class Vehicle():
    def __init__(self, simulationHandle):
        self.hC = VehicleCinematics(simulationHandle)
        self.hS = VehicleSensors(simulationHandle)
        self.map = Map()

    def changeCoord(self, direction):
        if direction == 'up':
            self.x += 1
        elif direction == 'right':
            self.y += 1
        elif direction == 'down':
            self.x -= 1
        elif direction == 'left':
            self.y -= 1

    def start(self):
        acumu = 0
        self.x = 0
        self.y = 0
        direction = 'up'
        self.distanceTraveled = 0
        while True:

            if self.distanceTraveled > 4.6:
                print(self.distanceTraveled)
                self.distanceTraveled = 0
                self.changeCoord(direction)
                self.map.addStraightSegment(self.x, self.y, direction)

            if self.hS.checkRight() and acumu > 20:
                print('r')
                self.hC.turn_straight_right(5)
                self.distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                self.map.addCornerSegmentRight(self.x, self.y, direction)
                if direction == 'up':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'up'

            elif self.hS.checkLeft() and acumu > 20:
                print('l')
                self.hC.turn_straight_left(5)
                self.distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                self.map.addCornerSegmentLeft(self.x, self.y, direction)
                if direction == 'up':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'up'

            elif self.hS.checkFront() and acumu > 20:
                print('rev')
                self.hC.turn_straight_right(5)
                self.hC.turn_straight_right(5)
                self.distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                self.map.addDeadEndSegment(self.x, self.y, direction)
                if direction == 'up':
                    direction = 'back'
                elif direction == 'back':
                    direction = 'up'
                elif direction == 'left':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'left'

            else:
                # if acumu > 10:
                self.correctWallDistDiff()
                self.hC.forward(5, 0.1)
                self.distanceTraveled += 0.1
                acumu += 1

    def correctWallDistDiff(self):
        if self.hS.checkWallDistDiffLeft():
            self.hC.turn_left(5, 7)
            self.hC.forward(5, 0.25)
            self.distanceTraveled += 0.15
            self.hC.turn_right(5, 5)
            # self.hC.to_right_angle_from_right(5)
        if self.hS.checkWallDistDiffRight():
            self.hC.turn_right(5, 7)
            self.hC.forward(5, 0.25)
            self.distanceTraveled += 0.15
            self.hC.turn_left(5, 5)
            # self.hC.to_right_angle_from_right(5)


def load_graphics(path):
    graphics = []
    # list_of_files=natsort.natsorted(listdir(path),reverse=False)
    #list_of_files = sorted(listdir(path),key=lambda x: int(os.path.splitext(x)[0]))
    list_of_files = [f for f in listdir(path) if isfile(join(path, f))]

    for i in list_of_files:
        # print(i)
        graphics.append(pygame.image.load(path+"/"+i))

    return graphics


def draw_window():
    pygame.init()
    DISPLAYSURF = pygame.display.set_mode((500, 500))
    pygame.display.set_caption('Hello World!')
    global maze_map
    rows = maze_map.shape[0]
    cols = maze_map.shape[1]
    graphics = load_graphics("map_segments")
    while True:  # main game loop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        i = 0
        j = 40*rows
        for x in range(0, rows):
            j = 40*rows
            for y in range(0, cols):
                if maze_map[y][x] != 0:
                    DISPLAYSURF.blit(graphics[maze_map[y][x]], (i, j))
                j -= 40
            i += 40
        pygame.display.update()
        time.sleep(0.5)


'''
if __name__ == "__main__":
    simulationHandle = Communication()
    simulationHandle.start()
    robot = Vehicle(simulationHandle)
    # robot.start()
    threading.Thread(target=robot.start).start()
    threading.Thread(target=draw_window).start()
'''
openConverterTemplateFile()
