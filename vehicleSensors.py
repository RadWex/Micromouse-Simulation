import sim
import math


class VehicleSensors():
    def __init__(self, simulationHandle):
        self.simulation = simulationHandle.simulation
        # init sensors
        err_code, self.ps_handle_rightSensor = sim.simxGetObjectHandle(
            self.simulation, "RightUS", sim.simx_opmode_blocking)
        err_code, self.detectionState_Right, self.detectedPoint_right, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_rightSensor, sim.simx_opmode_streaming)

        err_code, self.ps_handle_Left = sim.simxGetObjectHandle(
            self.simulation, "LeftUS", sim.simx_opmode_blocking)
        err_code, self.detectionState_Left, self.detectedPoint_left, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_Left, sim.simx_opmode_streaming)

        err_code, self.ps_handle_front = sim.simxGetObjectHandle(
            self.simulation, "FrontUS", sim.simx_opmode_blocking)
        err_code, self.detectionState_front, self.detectedPoint_front, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_front, sim.simx_opmode_streaming)
        self.updateSensors()

    def updateSensors(self):
        err_code, self.detectionState_Right, self.detectedPoint_right, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_rightSensor, sim.simx_opmode_buffer)
        err_code, self.detectionState_Left, self.detectedPoint_left, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_Left, sim.simx_opmode_buffer)
        err_code, self.detectionState_front, self.detectedPoint_front, self.detectedObjectHandle, self.detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            self.simulation, self.ps_handle_front, sim.simx_opmode_buffer)

    # methods for driving
    def checkRight(self):
        self.updateSensors()
        if math.atan2(self.detectedPoint_right[0], self.detectedPoint_right[2]) > 0.7 and self.detectedPoint_right[2] > 0.4:
            return True
        return False

    def checkLeft(self):
        self.updateSensors()
        if math.atan2(self.detectedPoint_left[0], self.detectedPoint_left[2]) > 0.7 and self.detectedPoint_left[2] > 0.4:
            return True
        return False

    def checkFront(self):
        self.updateSensors()
        if self.detectedPoint_front[2] < 0.3 and self.detectedPoint_front[2] != 0:
            return True
        return False

    def checkWallDistDiffLeft(self):
        if self.detectedPoint_left[2] > self.detectedPoint_right[2]+0.15 and self.detectionState_Right and self.detectionState_Left:
            return True
        return False

    def checkWallDistDiffRight(self):
        if self.detectedPoint_right[2] > self.detectedPoint_left[2]+0.15 and self.detectionState_Right and self.detectionState_Left:
            return True
        return False

    # methods for map reconstruction
    def wallInFront(self):
        self.updateSensors()
        if self.detectionState_front:
            return False
        return True

    def wallRight(self):
        self.updateSensors()
        if math.atan2(self.detectedPoint_right[0], self.detectedPoint_right[2]) > 0.5 and self.detectedPoint_right[2] > 0.4:
            return True
        return False

    def wallLeft(self):
        self.updateSensors()
        if math.atan2(self.detectedPoint_left[0], self.detectedPoint_left[2]) > 0.5 and self.detectedPoint_left[2] > 0.4:
            return True
        return False
