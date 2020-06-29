import sim
import math


class VehicleCinematics():
    def __init__(self, simulationHandle):
        self.simulation = simulationHandle.simulation
        # motors
        err_code, self.lb_motor_handle = sim.simxGetObjectHandle(
            self.simulation, "LeftBackMotor", sim.simx_opmode_blocking)
        err_code, self.rb_motor_handle = sim.simxGetObjectHandle(
            self.simulation, "RightBackMotor", sim.simx_opmode_blocking)
        err_code, self.lf_motor_handle = sim.simxGetObjectHandle(
            self.simulation, "LeftFrontMotor", sim.simx_opmode_blocking)
        err_code, self.rf_motor_handle = sim.simxGetObjectHandle(
            self.simulation, "RightFrontMotor", sim.simx_opmode_blocking)

        # wall for gyroscope
        err_code, self.rf_wall_handle = sim.simxGetObjectHandle(
            self.simulation, "240cmHighWall400cm0", sim.simx_opmode_blocking)

    def forward(self, speed, distance):
        if speed <= 0.01:
            return
        res, previousPosition = sim.simxGetFloatSignal(
            self.simulation, "signal", sim.simx_opmode_streaming)
        drivenDistance = 0
        distance = distance + previousPosition
        while drivenDistance < distance:
            res, drivenDistance = sim.simxGetFloatSignal(
                self.simulation, "signal", sim.simx_opmode_streaming)
            self.setMotors(-speed, -speed)
        self.setMotors(0, 0)

    def backward(self, speed, distance):
        if speed <= 0.01:
            return
        res, previousPosition = sim.simxGetFloatSignal(
            self.simulation, "signal", sim.simx_opmode_streaming)
        drivenDistance = 0
        distance = previousPosition - distance
        while drivenDistance > distance:
            res, drivenDistance = sim.simxGetFloatSignal(
                self.simulation, "signal", sim.simx_opmode_streaming)
            self.setMotors(speed, speed)
        self.setMotors(0, 0)

    def turn_left(self, speed, angle):
        if speed <= 0.01:
            return

        rad = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                           self.rf_wall_handle, sim.simx_opmode_blocking)
        old = rad[1][1]*(180/math.pi)
        obrocono = 0

        while True:
            if(obrocono >= angle):
                break
            self.setMotors(-speed, speed)
            rad = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                               self.rf_wall_handle, sim.simx_opmode_blocking)
            odczytana = rad[1][1]*(180/math.pi)
            monotonicznosc = old-odczytana
            if(monotonicznosc >= 0):
                obrocono -= odczytana - old
            else:
                obrocono += odczytana - old
            old = odczytana

        # print(obrocono)
        self.setMotors(0, 0)

    def turn_right(self, speed, angle):
        if speed <= 0.01:
            return

        rad = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                           self.rf_wall_handle, sim.simx_opmode_blocking)
        old = rad[1][1]*(180/math.pi)
        obrocono = 0

        while True:
            if(obrocono >= angle):
                break
            self.setMotors(speed, -speed)
            rad = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                               self.rf_wall_handle, sim.simx_opmode_blocking)
            odczytana = rad[1][1]*(180/math.pi)
            monotonicznosc = old-odczytana
            if(monotonicznosc <= 0):
                obrocono += odczytana - old
            else:
                obrocono -= odczytana - old
            old = odczytana

        self.setMotors(0, 0)

    def turn_straight_right(self, speed):
        if speed <= 0.01:
            return
        old = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                           self.rf_wall_handle, sim.simx_opmode_blocking)
        while True:
            self.setMotors(speed, -speed)
            new = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                               self.rf_wall_handle, sim.simx_opmode_blocking)
            if abs(old[1][1] - new[1][1]) > 1:
                if(new[1][1] > 1.52):
                    break
                elif(new[1][1] < -1.52):
                    break
                elif(new[1][1] < 0.05 and new[1][1] > -0.05):
                    break
        self.setMotors(0, 0)

    def turn_straight_left(self, speed):
        if speed <= 0.01:
            return
        old = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                           self.rf_wall_handle, sim.simx_opmode_blocking)
        while True:
            self.setMotors(-speed, speed)
            new = sim.simxGetObjectOrientation(self.simulation, self.lf_motor_handle,
                                               self.rf_wall_handle, sim.simx_opmode_blocking)
            if abs(old[1][1] - new[1][1]) > 1:
                if(new[1][1] > 1.52):
                    break
                elif(new[1][1] < -1.52):
                    break
                elif(new[1][1] < 0.05 and new[1][1] > -0.05):
                    break
        self.setMotors(0, 0)

    def setMotors(self, left, right):
        err_code = sim.simxSetJointTargetVelocity(
            self.simulation, self.rb_motor_handle, right, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(
            self.simulation, self.rf_motor_handle, right, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(
            self.simulation, self.lb_motor_handle, left, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(
            self.simulation, self.lf_motor_handle, left, sim.simx_opmode_streaming)
