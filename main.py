import sim
import sys
import math
from communication import Communication
from vehicleCinematics import VehicleCinematics
from vehicleSensors import VehicleSensors


class Map():
    def __init__(self):
        self.coord = (0, 0)
        self.interchange = []


class Vehicle():
    def __init__(self, simulationHandle):
        self.hC = VehicleCinematics(simulationHandle)
        self.hS = VehicleSensors(simulationHandle)

    def start(self):
        acumu = 0
        while True:
            if self.hS.checkRight() and acumu > 20:
                print('r')
                self.hC.turn_straight_right(5)
                acumu = 0
            elif self.hS.checkLeft() and acumu > 20:
                print('l')
                self.hC.turn_straight_left(5)
                acumu = 0
            elif self.hS.checkFront() and acumu > 20:
                print('rev')
                self.hC.turn_straight_right(5)
                self.hC.turn_straight_right(5)
                acumu = 0
            else:
                self.correctWallDistDiff()
                self.hC.forward(5, 0.1)
                acumu += 1

    def correctWallDistDiff(self):
        if self.hS.checkWallDistDiffLeft():
            self.hC.turn_left(5, 7)
            self.hC.forward(5, 0.3)
            self.hC.turn_right(5, 5)
        if self.hS.checkWallDistDiffRight():
            self.hC.turn_right(5, 7)
            self.hC.forward(5, 0.3)
            self.hC.turn_left(5, 5)


if __name__ == "__main__":
    simulationHandle = Communication()
    simulationHandle.start()
    robot = Vehicle(simulationHandle)
    robot.start()
