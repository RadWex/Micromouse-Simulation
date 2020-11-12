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
import mapViewer


maze_map = numpy.full((10, 10), 300, dtype=int)
simulation_ended = False

def segmentInserter(maze_map_big, segment, i, j):
    i *= 3
    j *= 3
    k = 0
    l = 0
    for u in range(i, i+3):
        for y in range(j, j+3):
            maze_map_big[u][y] = segment[l][k]
            k += 1
        k = 0
        l += 1


def openConverterTemplateFile(start, end):
    start = (start[0]*3+1, start[1]*3+1)
    end = (end[0]*3+1, end[1]*3+1)
    maze_map = numpy.loadtxt('map_raw.txt', delimiter=",", dtype=int)
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
            converterSegment.append(tmp2)
            tmp2 = []

    rows = maze_map.shape[0]
    cols = maze_map.shape[1]
    map_after_convert = numpy.ones((30, 30), dtype=int)
    for i in range(0, rows):
        for j in range(0, cols):
            if maze_map[i][j] != 300:
                segmentInserter(map_after_convert,
                                converterSegment[maze_map[i][j]], i, j)
    print(map_after_convert)
    print('---/n', start, end, '---')
    path = astar(map_after_convert, start, end)
    print(path)

    backToTheFuture(map_after_convert, converterSegment, path)


def aStarPathFinding(map_after_convert):
    path = astar(map_after_convert, (28, 1), (28, 4))
    return path


def backToTheFuture(maze_map, segments, path):
    rows = maze_map.shape[0]
    cols = maze_map.shape[1]
    map_after_convert = numpy.zeros((10, 10), dtype=int)
    flag = False
    for i in range(0, rows, 3):
        for j in range(0, cols, 3):
            for k in path:
                (x, y) = k
                if (x - 1) == i and (y - 1) == j:
                    flag = True
            map_after_convert[int(i / 3)][int(j / 3)
                                          ] = valInserter(maze_map, segments, i, j, flag)
            flag = False
        
    numpy.savetxt('best_way.txt', map_after_convert, fmt='%i', delimiter=',')
    return map_after_convert


def valInserter(maze_map_big, segments, i, j, flag):
    tmp = []
    segment = []
    for k in range(i, i + 3):
        for l in range(j, j + 3):
            tmp.append(maze_map_big[k][l])
        segment.append(tmp)
        tmp = []

    for index, k in enumerate(segments):
        if k == segment:
            if flag:
                return index + 14
            else:
                return index
    return 300


class Map():
    def __init__(self):
        self.interchange = []
        self.maze_map_ref = maze_map
        self.maze_map_ref[0][0] = 12

    def saveMapToFile(self):
        numpy.savetxt('map_raw.txt', 
            self.maze_map_ref, fmt='%i', delimiter=',')
        print(self.interchange)

    def addStraightSegment(self, x, y, direction):
        if direction == "up" or direction == "down":
            self.maze_map_ref[x][y] = 5
        else:
            self.maze_map_ref[x][y] = 4
        self.saveMapToFile()

    def addCornerSegmentRight(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 7
        elif direction == "right":
            self.maze_map_ref[x][y] = 6
        elif direction == "down":
            self.maze_map_ref[x][y] = 8
        elif direction == "left":
            self.maze_map_ref[x][y] = 9
        self.saveMapToFile()

    def addCrossSegmentRight(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 3
        elif direction == "right":
            self.maze_map_ref[x][y] = 0
        elif direction == "down":
            self.maze_map_ref[x][y] = 1
        elif direction == "left":
            self.maze_map_ref[x][y] = 2
        self.saveMapToFile()
        if (x, y) not in self.interchange:
            self.interchange.append((x, y))

    def addCornerSegmentFront(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 0
        elif direction == "right":
            self.maze_map_ref[x][y] = 1
        elif direction == "down":
            self.maze_map_ref[x][y] = 2
        elif direction == "left":
            self.maze_map_ref[x][y] = 3
        self.saveMapToFile()
        if (x, y) not in self.interchange:
            self.interchange.append((x, y))

    def addCornerSegmentLeft(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 6
        elif direction == "right":
            self.maze_map_ref[x][y] = 8
        elif direction == "down":
            self.maze_map_ref[x][y] = 9
        elif direction == "left":
            self.maze_map_ref[x][y] = 7
        self.saveMapToFile()

    def addCrossSegmentLeft(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 1
        elif direction == "right":
            self.maze_map_ref[x][y] = 2
        elif direction == "down":
            self.maze_map_ref[x][y] = 3
        elif direction == "left":
            self.maze_map_ref[x][y] = 0
        self.saveMapToFile()
        if (x, y) not in self.interchange:
            self.interchange.append((x, y))

    def addDeadEndSegment(self, x, y, direction):
        if direction == "up":
            self.maze_map_ref[x][y] = 10
        elif direction == "right":
            self.maze_map_ref[x][y] = 11
        elif direction == "down":
            self.maze_map_ref[x][y] = 12
        elif direction == "left":
            self.maze_map_ref[x][y] = 13
        self.saveMapToFile()

    def right_was_visited(self, x, y, direction):
        #print(x,y)
        self.del_visited_interchanges()
        if direction == 'right' and (x, y) in self.interchange:
            if self.maze_map_ref[x-1][y] != 300: 
                return True
        elif direction == 'left' and (x, y) in self.interchange:
            if self.maze_map_ref[x+1][y] != 300: 
                return True
        elif direction == 'up' and (x, y) in self.interchange:
            if self.maze_map_ref[x][y+1] != 300: 
                return True
        elif direction == 'down' and (x, y) in self.interchange:
            if self.maze_map_ref[x][y-1] != 300: 
                return True
        return False

    def left_was_visited(self, x, y, direction):
        #print(x,y)
        self.del_visited_interchanges()
        if direction == 'right' and (x, y) in self.interchange:
            if self.maze_map_ref[x+1][y] != 300: 
                return True
        elif direction == 'left' and (x, y) in self.interchange:
            if self.maze_map_ref[x-1][y] != 300: 
                return True
        elif direction == 'up' and (x, y) in self.interchange:
            if self.maze_map_ref[x][y-1] != 300: 
                return True
        elif direction == 'down' and (x, y) in self.interchange:
            if self.maze_map_ref[x][y+1] != 300: 
                return True
        return False

    def del_visited_interchanges(self):
        for x,y in self.interchange:
            if self.maze_map_ref[x][y]==0:
                if self.maze_map_ref[x-1][y]!=300 and\
                   self.maze_map_ref[x][y-1]!=300 and\
                   self.maze_map_ref[x][y+1]!=300:
                    self.interchange.remove((x,y))
            elif self.maze_map_ref[x][y]==1:
                if self.maze_map_ref[x][y-1]!=300 and\
                   self.maze_map_ref[x+1][y]!=300 and\
                   self.maze_map_ref[x-1][y]!=300:
                    self.interchange.remove((x,y))
            elif self.maze_map_ref[x][y]==2:
                if self.maze_map_ref[x+1][y]!=300 and\
                   self.maze_map_ref[x][y-1]!=300 and\
                   self.maze_map_ref[x][y+1]!=300:
                    self.interchange.remove((x,y))
            elif self.maze_map_ref[x][y]==3:
                if self.maze_map_ref[x-1][y]!=300 and\
                   self.maze_map_ref[x+1][y]!=300 and\
                   self.maze_map_ref[x][y+1]!=300:
                    self.interchange.remove((x,y))
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

    def start(self, x, y, direction):
        acumu = 0
        self.x = x
        self.y = y
        pass_flag = False
        direction = direction
        self.distanceTraveled = 0
        while True:

            if self.distanceTraveled > 5:
                print(self.x, " ", self.y)
                print(self.distanceTraveled)
                self.distanceTraveled = 0
                self.changeCoord(direction)
                self.map.addStraightSegment(self.x, self.y, direction)
            
            if self.hS.checkRight() and acumu > 20:
                print('r, ', self.x, " ", self.y)
                if self.hS.wallInFront():
                    flag_front = True
                else:
                    flag_front = False
                if self.hS.wallLeft():
                    flag_left = False
                else:
                    flag_left = True

                self.distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                if flag_left:
                    self.map.addCornerSegmentFront(self.x, self.y, direction)

                elif flag_front:
                    self.map.addCrossSegmentRight(self.x, self.y, direction)
                else:
                    self.map.addCornerSegmentRight(self.x, self.y, direction)

                if not self.map.right_was_visited(self.x, self.y, direction):
                    self.hC.turn_straight_right(5)
                    if direction == 'up':
                        direction = 'right'
                    elif direction == 'right':
                        direction = 'down'
                    elif direction == 'down':
                        direction = 'left'
                    elif direction == 'left':
                        direction = 'up'
                else:
                    pass_flag = True
                self.distanceTraveled = -0.4
                acumu = 0

            elif self.hS.checkLeft() and (acumu > 20 or pass_flag):
                print('l, ', self.x, " ", self.y)
                
                if self.hS.wallInFront():
                    flag_front = True
                else:
                    flag_front = False
                if self.hS.wallRight():
                    flag_right = False
                else:
                    flag_right = True

                if not pass_flag:
                    self.distanceTraveled = 0
                    acumu = 0
                    self.changeCoord(direction)

                    if flag_right:
                        self.map.addCornerSegmentFront(self.x, self.y, direction)
                    elif flag_front:
                        self.map.addCrossSegmentLeft(self.x, self.y, direction)
                    else:
                        self.map.addCornerSegmentLeft(self.x, self.y, direction)
                
                if not self.map.left_was_visited(self.x, self.y, direction):
                    self.hC.turn_straight_left(5)
                    if direction == 'up':
                        direction = 'left'
                    elif direction == 'left':
                        direction = 'down'
                    elif direction == 'down':
                        direction = 'right'
                    elif direction == 'right':
                        direction = 'up'
                self.distanceTraveled = -0.4
                acumu = 0
                pass_flag = False


            elif self.hS.checkFront() and acumu > 20:
                print('rev, ', self.x, " ", self.y)
                self.distanceTraveled = 0
                acumu = 0
                self.hC.turn_straight_right(5)
                self.hC.turn_straight_right(5)
                self.distanceTraveled = -0.6
                acumu = 0
                self.changeCoord(direction)
                self.map.addDeadEndSegment(self.x, self.y, direction)
                if direction == 'up':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'up'
                elif direction == 'left':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'left'
                if not self.map.interchange:
                    print("Skanowanie zakonczone")
                    
                    workWithMap((self.x, self.y), (0, 0))
                    global maze_map
                    maze_map = numpy.loadtxt('best_way.txt', delimiter=",", dtype=int)
                    self.road_to_objective(direction)
                    break

            else:
                ################
                # if acumu > 10:
                self.correctWallDistDiff()
                self.hC.forward(5, 0.1)
                self.distanceTraveled += 0.1
                acumu += 1

    def road_to_objective(self, direction):
        acumu = 0
        direction = direction
        distanceTraveled = 0
        while True:

            if distanceTraveled > 5:
                print(self.x, " ", self.y)
                print(distanceTraveled)
                distanceTraveled = 0
                self.changeCoord(direction)
            
            if self.road_to_objective_right(self.x, self.y, direction) and acumu > 20:
                print('r, ', self.x, " ", self.y)

                distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)

                self.hC.turn_straight_right(5)
                if direction == 'up':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'up'
                distanceTraveled = -0.4
                acumu = 0

            elif self.road_to_objective_left(self.x, self.y, direction) and acumu > 20:
                print('l, ', self.x, " ", self.y)

                distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                
                self.hC.turn_straight_left(5)
                if direction == 'up':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'up'
                distanceTraveled = -0.4
                acumu = 0

            elif self.road_to_right_hmm(self.x, self.y, direction) and acumu > 20:
                distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                
                self.hC.turn_straight_right(5)
                if direction == 'up':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'up'
                distanceTraveled = -0.4
                acumu = 0

            elif self.road_to_left_hmm(self.x, self.y, direction)  and acumu > 20:
                distanceTraveled = 0
                acumu = 0
                self.changeCoord(direction)
                
                self.hC.turn_straight_left(5)
                if direction == 'up':
                    direction = 'left'
                elif direction == 'left':
                    direction = 'down'
                elif direction == 'down':
                    direction = 'right'
                elif direction == 'right':
                    direction = 'up'
                distanceTraveled = -0.4
                acumu = 0

            else:
                ################
                # if acumu > 10:
                self.correctWallDistDiff()
                self.hC.forward(5, 0.1)
                distanceTraveled += 0.1
                acumu += 1

    def road_to_objective_right(self, x, y, direction):
        global maze_map
        if direction == 'right':
            if maze_map[x-1][y] == 15: 
                return True
        elif direction == 'left':
            if maze_map[x+1][y] == 17: 
                return True
        elif direction == 'up':
            if maze_map[x][y+1] == 14: 
                return True
        elif direction == 'down':
            if maze_map[x][y-1] == 16: 
                return True
        return False

    def road_to_objective_left(self, x, y, direction):
        global maze_map
        if direction == 'right':
            if maze_map[x-1][y] == 17: 
                return True
        elif direction == 'left':
            if maze_map[x+1][y] == 15: 
                return True
        elif direction == 'up':
            if maze_map[x][y+1] == 16: 
                return True
        elif direction == 'down':
            if maze_map[x][y-1] == 14: 
                return True
        return False

    def road_to_left_hmm(self, x, y, direction):
        global maze_map
        if direction == 'right':
            if maze_map[x][y] == 22: 
                return True
        elif direction == 'left':
            if maze_map[x][y] == 21: 
                return True
        elif direction == 'up':
            if maze_map[x][y] == 20: 
                return True
        elif direction == 'down':
            if maze_map[x][y] == 23: 
                return True
        return False

    def road_to_right_hmm(self, x, y, direction):
        global maze_map
        if direction == 'right':
            if maze_map[x][y] == 20: 
                return True
        elif direction == 'left':
            if maze_map[x][y] == 23: 
                return True
        elif direction == 'up':
            if maze_map[x][y] == 21: 
                return True
        elif direction == 'down':
            if maze_map[x][y] == 22: 
                return True
        return False

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
    list_of_files = [f for f in listdir(path) if isfile(join(path, f))]

    for i in list_of_files:
        graphics.append(pygame.image.load(path+"/"+i))

    return graphics


def draw_window():
    pygame.init()
    DISPLAYSURF = pygame.display.set_mode((500, 500))
    pygame.display.set_caption('Simulation')
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
                if maze_map[y][x] != 300:
                    DISPLAYSURF.blit(graphics[maze_map[y][x]], (i, j))
                j -= 40
            i += 40
        pygame.display.update()
        time.sleep(0.5)
        if simulation_ended:
            break


def drive(start, direction):
    simulationHandle = Communication()
    simulationHandle.start()
    robot = Vehicle(simulationHandle)
    threading.Thread(target=robot.start, args=(
        start[0], start[1], direction)).start()
    threading.Thread(target=draw_window).start()


def workWithMap(start, end):
    openConverterTemplateFile(start, end)


if __name__ == "__main__":
    #drive((5, 5), "up")
    #drive((2, 5), "left")
    
    drive((0, 0), "up")
    #start = (3, 0)
    #end = (0, 0)
    #workWithMap(start, end)
    #mapViewer.draw_window('best_way.txt')
