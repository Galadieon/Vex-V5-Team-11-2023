# ------------------------------------------
#
# 	Project:      Mecanum Robot Program
#	Author:       Lam Ninh & Kaily Do
#	Created:      11/1/2022
#	Description:  VEXcode V5 Python Mecanum
#                 Project
#
# ------------------------------------------

# Library imports
from vex import *

import math

# Begin project code
brain = Brain()

# Robot variables and configuration code

# BAD PORT: 5, 6, 7, 8, 11, 12

F1 = Motor(Ports.PORT5, GearSetting.RATIO_6_1,
           False)  # motor closest to flywheel
# F2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
# FM = MotorGroup(flywheelLeft, flywheelRight)

# Controller variables

controllerEnabled = True
controller = Controller(PRIMARY)

# Autonomous paths

#        x,  y, Θ, driveVel, turnVel
path1 = [[0.0, 10.0, math.pi / 2, 25, 25]]

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# ---------------------------CONTROLLER LOOP---------------------------


# Controller loop to handle controller readings
def controllerLoop():
    global controllerEnabled, controller, drivetrain
    deadZoneVal = axisCurve(0.1)

    printThread = Thread(printToController)

    while (True):
        if controllerEnabled:
            forward = axisCurve(controller.axis3.position())
            strafe = axisCurve(controller.axis4.position())
            rotate = axisCurve(controller.axis1.position())

            if abs(forward) > deadZoneVal or abs(strafe) > deadZoneVal or abs(
                    rotate) > deadZoneVal:
                drivetrain.drive(forward * (drivetrain.driveVel / 100),
                                 strafe * (drivetrain.driveVel / 100),
                                 rotate * (drivetrain.turnVel / 100))
            else:
                drivetrain.stop()

        wait(10, MSEC)


def printToController():
    global drivetrain
    while (True):
        # controller.screen.print("Right Encoder: ", drivetrain.rightEncoder.value())
        # controller.screen.next_row()
        # controller.screen.print("Left Encoder: ", drivetrain.leftEncoder.value())
        # controller.screen.next_row()
        # controller.screen.print("Aux Encoder: ", drivetrain.auxEncoder.value())
        # controller.screen.next_row()

        controller.screen.print("Global X: ", drivetrain.x)
        controller.screen.next_row()
        controller.screen.print("Global Y: ", drivetrain.y)
        controller.screen.next_row()
        controller.screen.print("Global Θ: ", math.degrees(drivetrain.Θ))
        controller.screen.next_row()

        wait(100, MSEC)

        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)


def axisCurve(x):
    return (x**3) / 10000
    # if x > 0: return (x ** 2) / 100
    # return (x ** 2) / -100


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS


def non_competition_start():
    Driver_Control()


def Default_Motor_Speed():
    global drivetrain
    drivetrain.set_drive_velocity(100, VelocityUnits.PERCENT)
    drivetrain.set_turn_velocity(100, VelocityUnits.PERCENT)
    drivetrain.set_stopping(COAST)


# AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS


def vexcode_auton_function():
    global drivetrain, F1
    auton_task_0 = Thread(Autonomous_Control)
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)
    drivetrain.stop()
    F1.stop()
    auton_task_0.stop()


def Autonomous_Control():
    global path1, drivetrain, brain
    brain.screen.print("Starting auto")
    drivetrain.startAuto(path1)


# DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS


def vexcode_driver_function():
    global drivetrain, F1
    driver_control_task_0 = Thread(Driver_Control)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    drivetrain.stop()
    F1.stop()
    driver_control_task_0.stop()


def Driver_Control():
    Default_Motor_Speed()


# ---------------------------BUTTON FUNCTIONS---------------------------


def L1_Pressed():
    pass


def L2_Pressed():
    if drivetrain.turnVel == 100: drivetrain.set_turn_velocity(50)
    else: drivetrain.set_turn_velocity(100)


def R1_Pressed():
    pass


def R2_Pressed():
    if drivetrain.driveVel == 100: drivetrain.set_drive_velocity(50)
    else: drivetrain.set_drive_velocity(100)


def A_Pressed():
    global drivetrain, path1
    drivetrain.startAuto(path1)


def B_Pressed():
    if drivetrain.motorMode == BRAKE: drivetrain.set_stopping(COAST)
    elif drivetrain.motorMode == COAST: drivetrain.set_stopping(BRAKE)


def X_Pressed():
    drivetrain.resetOdom()


def Y_Pressed():
    pass


def Up_Pressed():
    drivetrain.goBackToOG()


def Down_Pressed():
    pass


def Left_Pressed():
    pass


def Right_Pressed():
    pass


# ---------------------------HELPER FUNCTIONS---------------------------


def throwTheThings():
    global F1
    F1.set_velocity(100, PERCENT)
    F1.spin(FORWARD)

    wait(1, SECONDS)
    F1.stop()


# def findAvg

# def tanh(x, max): # x is in inches
#     n = 1.732
#     return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x))) * max


def tanh(x):  # x is in inches
    n = 1.732
    return ((n**x) - (n**(-x))) / ((n**x) + (n**(-x)))


# def tanhTurning(x, max): # x is in radians
#     n = 23.2742 # seems a little too large for turn motor value, try 3
#     return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x))) * max


def tanhTurning(x):  # x is in radians
    n = 23.2742  # seems a little too large for turn motor value, try 3
    return ((n**x) - (n**(-x))) / ((n**x) + (n**(-x)))


def PID(encoder, target, integral, previousError, minimumVoltage, Kp, Ki, Kd):
    current = encoder.velocity(RPM)

    error = target - current

    integral += error
    if error == 0: integral = 0
    if abs(error) > 10: integral = 0

    derivative = error - previousError
    previousError = error

    motorVoltage = minimumVoltage + abs((Kp * error) + (Ki * integral) +
                                        (Kd * derivative))

    if motorVoltage > 12: motorVoltage = 12

    return integral, previousError, motorVoltage


# ---------------------------MECANUM DRIVETRAIN---------------------------


class MecDriveTrain:

    def __init__(self, wheelTravel, trackWidth, wheelBase, unit, gearRatio):
        self.FL = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
        self.FR = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
        self.BR = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)                                                                    
        self.BL = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)

        self.rightEncoder = Encoder(brain.three_wire_port.e)
        self.leftEncoder = Encoder(brain.three_wire_port.a)
        self.auxEncoder = Encoder(brain.three_wire_port.c)

        self.L = 13.5
        self.B = 5.0
        self.D = 2.75
        self.N = 360
        self.circumference = math.pi * self.D
        self.inchsPerTick = self.circumference / self.N

        self.wheelTravel = wheelTravel
        self.trackWidth = trackWidth
        self.wheelBase = wheelBase
        self.unit = unit
        self.gearRatio = gearRatio
        self.driveVel = 0
        self.turnVel = 0
        self.x = 0
        self.y = 0
        self.Θ = math.pi / 2
        self.motorMode = COAST

        self.currRightVal = 0  # current encoder value for right wheel
        self.currLeftVal = 0  # current encoder value for left wheel
        self.currAuxVal = 0  # current encoder value for back wheel

        self.prevRightVal = 0  # previous encoder value for right wheel
        self.prevLeftVal = 0  # previous encoder value for left wheel
        self.prevAuxVal = 0  # previous encoder value for back wheel

        wait(10, MSEC)

        self.odomThread = Thread(self.updatePosition)

    # ---------------------------AUTO AND ODOMETRY---------------------------

    def startAuto(self, path):
        drivetrain.set_stopping(COAST)

        atPosition = False
        for step in path:
            while not atPosition:
                atPosition = drivetrain.drive_to(step[0], step[1], step[2],
                                                 step[3], step[4])

    def updatePosition(self):
        while (True):
            # start = brain.timer.time(MSEC)
            self.prevRightVal = self.currRightVal
            self.prevLeftVal = self.currLeftVal
            self.prevAuxVal = self.currAuxVal

            self.currRightVal = self.rightEncoder.value()
            self.currLeftVal = self.leftEncoder.value()
            self.currAuxVal = self.auxEncoder.value()

            dn2 = self.currRightVal - self.prevRightVal
            dn1 = self.currLeftVal - self.prevLeftVal
            dn3 = self.currAuxVal - self.prevAuxVal

            dtheta = self.inchsPerTick * ((dn2 - dn1) / self.L)
            dx = self.inchsPerTick * ((dn1 + dn2) / 2.0)
            dy = self.inchsPerTick * (dn3 - ((dn2 - dn1) * (self.B / self.L)))

            theta = self.Θ + (dtheta / 2.0)
            self.x -= -dx * math.cos(-theta) + dy * math.sin(-theta)
            self.y += -dx * math.sin(-theta) - dy * math.cos(-theta)
            self.Θ += dtheta

            wait(100, MSEC)

            # allows for faster
            # while brain.timer.time(MSEC) - start < 10:
                # may need wait(1, MSEC) or something similar
                # pass

    # ---------------------------DRIVE FUNCTIONS---------------------------

    def drive_to(self, xTarget: float, yTarget: float, ΘTarget: float,
                 driveVel: float, turnVel: float):
        deltaX, deltaY = self.calcLocalXY(xTarget, yTarget)
        deltaTheta = ΘTarget - self.Θ

        if abs(deltaX) < 0.25 and abs(deltaY) < 0.25 and abs(
                deltaTheta) < 0.035:
            print("AT POSITION")
            return True

        forward = 10 if deltaY > 1 else -10 if deltaY < -1 else 0
        strafe = 10 if deltaX > 1 else -10 if deltaX < -1 else 0
        rotate = 10 if deltaTheta > 0.045 else -10 if deltaTheta < -0.045 else 0

        print("\n\n\n\n", round(self.x, 1), "\t", round(self.y, 1), "\t", round(self.Θ, 1))
        print(round(deltaX, 1), "\t", round(deltaY, 1), "\t", round(deltaTheta, 1))

        self.autoDrive(forward, strafe, 0)

        return False

    def autoDrive(self, forward, strafe, rotate):
        self.FL.set_velocity(forward + strafe + rotate, PERCENT)
        self.FR.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
        self.BR.set_velocity(-forward - strafe + rotate, PERCENT)
        self.BL.set_velocity(forward - strafe + rotate, PERCENT)

        self.FL.spin_for(FORWARD, 10, MSEC, wait=False)
        self.FR.spin_for(FORWARD, 10, MSEC, wait=False)
        self.BR.spin_for(FORWARD, 10, MSEC, wait=False)
        self.BL.spin_for(FORWARD, 10, MSEC, wait=False)

    def drive(self, forward, strafe, rotate):
        self.FL.set_velocity(forward + strafe + rotate, PERCENT)
        self.FR.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
        self.BR.set_velocity(-forward - strafe + rotate, PERCENT)
        self.BL.set_velocity(forward - strafe + rotate, PERCENT)

        self.FL.spin(FORWARD)
        self.FR.spin(FORWARD)
        self.BR.spin(FORWARD)
        self.BL.spin(FORWARD)

    def goBackToOG(self):
        self.drive_to(0, 0, math.pi / 2, 25, 25)

    def stop(self):
        self.FL.stop()
        self.FR.stop()
        self.BR.stop()
        self.BL.stop()

    # ---------------------------HELPER FUNCTIONS---------------------------

    def calcLocalXY(self, xTarget: float, yTarget: float):
        deltaX = xTarget - self.x
        deltaY = yTarget - self.y

        dist = pow(pow(deltaX, 2) + pow(deltaY, 2), 1/2)

        targetTheta = math.atan2(deltaY, deltaX)
        localRelTheta = (targetTheta if targetTheta >= 0 else targetTheta +
                         (2 * math.pi)) - self.Θ + (math.pi / 2)

        localDeltaX = dist * math.cos(localRelTheta)
        localDeltaY = dist * math.sin(localRelTheta)

        # limit excessively long and small numbers
        localDeltaX = round(localDeltaX, 10) # 10.0000000000
        localDeltaY = round(localDeltaY, 10) # 10.0000000000

        print(localDeltaX, localDeltaY)

        return localDeltaX, localDeltaY

    def set_drive_velocity(self, velocity, units=VelocityUnits.RPM):
        self.driveVel = velocity

    def set_turn_velocity(self, velocity, units=VelocityUnits.RPM):
        self.turnVel = velocity

    def set_stopping(self, mode=BrakeType.COAST):
        self.motorMode = mode
        self.FL.set_stopping(mode)
        self.FR.set_stopping(mode)
        self.BR.set_stopping(mode)
        self.BL.set_stopping(mode)

    def resetOdom(self):
        self.odomThread.stop()

        self.x = 0
        self.y = 0
        self.Θ = math.pi / 2

        # self.odomThread = Thread(self.updatePosition)


# ---------------------------REQUIRED CODE---------------------------

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# system event handlers
controller.buttonL1.pressed(L1_Pressed)
controller.buttonL2.pressed(L2_Pressed)
controller.buttonR1.pressed(R1_Pressed)
controller.buttonR2.pressed(R2_Pressed)

controller.buttonA.pressed(A_Pressed)
controller.buttonB.pressed(B_Pressed)
controller.buttonX.pressed(X_Pressed)
controller.buttonY.pressed(Y_Pressed)

controller.buttonUp.pressed(Up_Pressed)
controller.buttonDown.pressed(Down_Pressed)
controller.buttonLeft.pressed(Left_Pressed)
controller.buttonRight.pressed(Right_Pressed)

drivetrain = MecDriveTrain(4 * math.pi, 14.097242, 11.5, INCHES, 36.0 / 84.0)

controllerThread = Thread(controllerLoop)

competition = Competition(vexcode_driver_function, vexcode_auton_function)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

non_competition_start()
