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
                Robot.drivetrain.drive(forward * (Robot.drivetrain.driveVel / 100),
                                 strafe * (Robot.drivetrain.driveVel / 100),
                                 rotate * (Robot.drivetrain.turnVel / 100))
            elif Robot.autoRoutine.autoIsRunning == True:
                pass
            else:
                Robot.drivetrain.stop()

        wait(10, MSEC)


def printToController():
    global controller, drivetrain
    while (True):
        # controller.screen.print("Right Encoder: ", Robot.drivetrain.rightEncoder.value())
        # controller.screen.next_row()
        # controller.screen.print("Left Encoder: ", Robot.drivetrain.leftEncoder.value())
        # controller.screen.next_row()
        # controller.screen.print("Aux Encoder: ", Robot.drivetrain.auxEncoder.value())
        # controller.screen.next_row()

        robotX, robotY, robotΘ = Robot.odometry.getPose()

        controller.screen.print("Global X: ", robotX)
        controller.screen.next_row()
        controller.screen.print("Global Y: ", robotY)
        controller.screen.next_row()
        controller.screen.print("Global Θ: ", math.degrees(robotΘ))
        controller.screen.next_row()

        wait(100, MSEC)

        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)


def axisCurve(x):
    return (pow(x, 3)) / 10_000
    # if x > 0: return (x ** 2) / 100
    # return (x ** 2) / -100


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS


def non_competition_start():
    Driver_Control()


def Default_Motor_Speed():
    global drivetrain
    Robot.drivetrain.set_drive_velocity(100, VelocityUnits.PERCENT)
    Robot.drivetrain.set_turn_velocity(100, VelocityUnits.PERCENT)
    Robot.drivetrain.set_stopping(COAST)


# AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS


def vexcode_auton_function():
    global drivetrain, F1
    auton_task_0 = Thread(Autonomous_Control)
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)
    Robot.drivetrain.stop()
    F1.stop()
    auton_task_0.stop()


def Autonomous_Control():
    global path1, drivetrain, brain
    brain.screen.print("Starting auto")
    Robot.autoRoutine.addAutoPaths(path1)
    Robot.autoRoutine.runAuto(0)


# DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS


def vexcode_driver_function():
    global drivetrain, F1
    driver_control_task_0 = Thread(Driver_Control)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    Robot.drivetrain.stop()
    F1.stop()
    driver_control_task_0.stop()


def Driver_Control():
    Default_Motor_Speed()


# ---------------------------BUTTON FUNCTIONS---------------------------


def L1_Pressed():
    pass


def L2_Pressed():
    if Robot.drivetrain.turnVel == 100: Robot.drivetrain.set_turn_velocity(50)
    else: Robot.drivetrain.set_turn_velocity(100)


def R1_Pressed():
    pass


def R2_Pressed():
    if Robot.drivetrain.driveVel == 100: Robot.drivetrain.set_drive_velocity(50)
    else: Robot.drivetrain.set_drive_velocity(100)


def A_Pressed():
    global drivetrain, path1
    if Robot.autoRoutine.autoIsRunning == False:
        Robot.autoRoutine.addAutoPaths(path1)
        Thread(Robot.autoRoutine.runAuto)
    else:
        Robot.autoRoutine.stopAuto()


def B_Pressed():
    if Robot.drivetrain.motorMode == BRAKE: Robot.drivetrain.set_stopping(COAST)
    elif Robot.drivetrain.motorMode == COAST: Robot.drivetrain.set_stopping(BRAKE)


def X_Pressed():
    Robot.odometry.resetPose()


def Y_Pressed():
    pass


def Up_Pressed():
    Robot.autoRoutine.goBackToOG()


def Down_Pressed():
    pass


def Left_Pressed():
    pass


def Right_Pressed():
    pass


# ---------------------------MECANUM DRIVETRAIN---------------------------

class Constants:
    LEFT_DRIVE_TRAIN_FORWARD = Ports.PORT1
    RIGHT_DRIVE_TRAIN_FORWARD = Ports.PORT2
    RIGHT_DRIVE_TRAIN_BACK = Ports.PORT10
    LEFT_DRIVE_TRAIN_BACK = Ports.PORT9

    WHEEL_TRAVEL = 4 * math.pi
    TRACK_WIDTH = 14.097242
    WHEEL_BASE = 11.5

    LEFT_RIGHT_ODOMETRY_DISTANCE = 13.5
    AUX_ODOMETRY_DISTANCE = 5.0
    ODOMETRY_DIAMETER = 2.75
    QUADRATURE_ENCODER_TICKS = 360
    ODOMETRY_CIRCUMFERENCE = math.pi * ODOMETRY_DIAMETER
    INCHES_PER_TICK = ODOMETRY_CIRCUMFERENCE / QUADRATURE_ENCODER_TICKS

class PID:
    def __init__(self, Kp = 0, Ki = 0, Kd = 0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prevError = 0.0
        self.integral = 0.0
        self.value = 0.0

    def update(self, target, current):
        error = target - current

        self.integral += error
        if error == 0 or abs(error) > 2.0: self.integral = 0.0

        derivative = error - self.previousError
        self.previousError = error

        self.value = abs((Kp * error) + (Ki * self.integral) + (Kd * derivative))

        if self.value > 100: self.value = 100
        if self.value < -100: self.value = -100

        return self.value

class Odometry:
    def __init__(self, rightEncoder, leftEncoder, auxEncoder):
        self.x = 0
        self.y = 0
        self.Θ = math.pi / 2

        self.rightEncoder = rightEncoder
        self.leftEncoder = leftEncoder
        self.auxEncoder = auxEncoder

        self.threadIsRunning = False
        self.threadIsPaused = False

    def start(self):
        Thread(self.updatePose)

    def updatePose(self):
        inchsPerTick = Constants.INCHES_PER_TICK
        LRDistance = Constants.LEFT_RIGHT_ODOMETRY_DISTANCE
        BDistance = Constants.AUX_ODOMETRY_DISTANCE

        currRightVal = 0  # current encoder value for right wheel
        currLeftVal = 0  # current encoder value for left wheel
        currAuxVal = 0  # current encoder value for back wheel
        
        prevRightVal = 0  # previous encoder value for right wheel
        prevLeftVal = 0  # previous encoder value for left wheel
        prevAuxVal = 0  # previous encoder value for back whee
        
        self.threadIsRunning = True

        while (self.threadIsRunning):
            if self.threadIsPaused: continue

            start = brain.timer.time(MSEC)

            prevRightVal = currRightVal
            prevLeftVal = currLeftVal
            prevAuxVal = currAuxVal

            currRightVal = self.rightEncoder.value()
            currLeftVal = self.leftEncoder.value()
            currAuxVal = self.auxEncoder.value()

            dn2 = currRightVal - prevRightVal
            dn1 = currLeftVal - prevLeftVal
            dn3 = currAuxVal - prevAuxVal

            dtheta = inchsPerTick * ((dn2 - dn1) / LRDistance)
            dx = inchsPerTick * ((dn1 + dn2) / 2.0)
            dy = inchsPerTick * (dn3 - ((dn2 - dn1) * (BDistance / LRDistance)))

            theta = self.Θ + (dtheta / 2.0)
            self.x -= -dx * math.cos(-theta) + dy * math.sin(-theta)
            self.y += -dx * math.sin(-theta) - dy * math.cos(-theta)
            self.Θ += dtheta

            while brain.timer.time(MSEC) - start < 10:
                continue

    def stop(self):
        self.threadIsRunning = False

    def resetPose(self):
        self.setPose(0, 0, math.pi / 2)

    def setPose(self, newX, newY, newΘ):
        self.threadIsPaused = True
        wait(0.5, MSEC)
        self.x = newX
        self.y = newY
        self.Θ = newΘ
        self.resetEncoders()
        self.threadIsPaused = False

    def getPose(self):
        return self.x, self.y, self.Θ

    def resetEncoders(self):
        self.rightEncoder.set_position(0, DEGREES)
        self.leftEncoder.set_position(0, DEGREES)
        self.auxEncoder.set_position(0, DEGREES)

class MecanumDriveTrain:
    def __init__(self, FL, FR, BR, BL):
        self.motorFrontLeft = Motor(FL, GearSetting.RATIO_18_1, False)
        self.motorFrontRight = Motor(FR, GearSetting.RATIO_18_1, False)
        self.motorBackRight = Motor(BR, GearSetting.RATIO_18_1, False)
        self.motorBackLeft = Motor(BL, GearSetting.RATIO_18_1, False)
        
        self.driveVel = 100
        self.turnVel = 100
        self.motorMode = COAST

        Robot.odometry.resetEncoders()
        Robot.odometry.start()

    def drive(self, forward, strafe, rotate):
        self.motorFrontLeft.set_velocity(forward + strafe + rotate, PERCENT)
        self.motorFrontRight.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
        self.motorBackRight.set_velocity(-forward - strafe + rotate, PERCENT)
        self.motorBackLeft.set_velocity(forward - strafe + rotate, PERCENT)

        self.motorFrontLeft.spin(FORWARD)
        self.motorFrontRight.spin(FORWARD)
        self.motorBackRight.spin(FORWARD)
        self.motorBackLeft.spin(FORWARD)

    def driveTo(self, xTarget: float, yTarget: float, ΘTarget: float,
                 driveVel: float, turnVel: float, localXY):
        deltaX, deltaY = localXY
        deltaTheta = ΘTarget - Robot.odometry.Θ

        if abs(deltaX) < 0.25 and abs(deltaY) < 0.25 and abs(
                deltaTheta) < 0.035:
            print("AT POSITION")
            return True

        forward = 10 if deltaY > 1 else -10 if deltaY < -1 else 0
        strafe = 10 if deltaX > 1 else -10 if deltaX < -1 else 0
        rotate = 10 if deltaTheta > 0.045 else -10 if deltaTheta < -0.045 else 0

        Robot.drivetrain.autoDrive(forward, strafe, rotate)

        return False

    def autoDrive(self, forward, strafe, rotate):
        self.motorFrontLeft.set_velocity(forward + strafe + rotate, PERCENT)
        self.motorFrontRight.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
        self.motorBackRight.set_velocity(-forward - strafe + rotate, PERCENT)
        self.motorBackLeft.set_velocity(forward - strafe + rotate, PERCENT)

        self.motorFrontLeft.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        self.motorFrontRight.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        self.motorBackRight.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        self.motorBackLeft.spin_for(FORWARD, 0.01, SECONDS, wait=False)

    def set_drive_velocity(self, velocity, units=VelocityUnits.RPM):
        self.driveVel = velocity

    def set_turn_velocity(self, velocity, units=VelocityUnits.RPM):
        self.turnVel = velocity

    def set_stopping(self, mode=BrakeType.COAST):
        self.motorMode = mode
        self.motorFrontLeft.set_stopping(mode)
        self.motorFrontRight.set_stopping(mode)
        self.motorBackRight.set_stopping(mode)
        self.motorBackLeft.set_stopping(mode)

    def stop(self):
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackRight.stop()
        self.motorBackLeft.stop()

class AutonomousRoutine:
    def __init__(self):
        self.autoIsRunning = False
        self.pathList = [[[]]]

    def addAutoPaths(self, pathList):
        self.pathList = pathList

    def runAuto(self, pathNumber):
        Robot.drivetrain.set_stopping(COAST)

        self.autoIsRunning = True

        atPosition = False
        for step in self.pathList[pathNumber]:
            while not atPosition:
                if self.autoIsRunning == False:
                    return
                atPosition = Robot.drivetrain.driveTo(step[0], step[1], step[2], step[3],
                                           step[4], self.calcLocalXY(step[0], step[1]))

    def stopAuto(self):
        self.autoIsRunning = False

    def calcLocalXY(self, xTarget, yTarget):
        robotX, robotY, robotΘ = Robot.odometry.getPose()

        deltaX = xTarget - robotX
        deltaY = yTarget - robotY

        # dist = math.hypot(deltaY, deltaX)
        dist = pow(pow(deltaX, 2) + pow(deltaY, 2), 1 / 2)

        targetTheta = math.atan2(deltaY, deltaX)
        localRelTheta = (targetTheta if targetTheta >= 0 else targetTheta +
                         (2 * math.pi)) - robotΘ + (math.pi / 2)

        localDeltaX = dist * math.cos(localRelTheta)
        localDeltaY = dist * math.sin(localRelTheta)

        # limit excessively long and small numbers
        localDeltaX = round(localDeltaX, 7)  # 10.0000000
        localDeltaY = round(localDeltaY, 7)  # 10.0000000

        return localDeltaX, localDeltaY

    def goBackToOG(self):
        Robot.drivetrain.driveTo(0, 0, math.pi / 2, 25, 25, self.calcLocalXY(0, 0))

class Robot:
    drivetrain = MecanumDriveTrain(
        Constants.LEFT_DRIVE_TRAIN_FORWARD,
        Constants.RIGHT_DRIVE_TRAIN_FORWARD,
        Constants.RIGHT_DRIVE_TRAIN_BACK,
        Constants.LEFT_DRIVE_TRAIN_BACK
    )

    odometry = Odometry(
                    Encoder(brain.three_wire_port.e),
                    Encoder(brain.three_wire_port.a),
                    Encoder(brain.three_wire_port.c)
    )

    indexer = Indexer()
    
    flywheel = Flywheel()

    intake = Intake()

    autoRoutine = AutonomousRoutine()

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

controllerThread = Thread(controllerLoop)

competition = Competition(vexcode_driver_function, vexcode_auton_function)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

non_competition_start()

# class MecanumDriveTrain:
    # def __init__(self, wheelTravel, trackWidth, wheelBase, unit, gearRatio):
    #     self.FL = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    #     self.FR = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
    #     self.BR = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
    #     self.BL = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
        
    #     self.rightEncoder = Encoder(brain.three_wire_port.e)
    #     self.leftEncoder = Encoder(brain.three_wire_port.a)
    #     self.auxEncoder = Encoder(brain.three_wire_port.c)

    #     self.wheelTravel = wheelTravel
    #     self.trackWidth = trackWidth
    #     self.wheelBase = wheelBase
    #     self.unit = unit
    #     self.gearRatio = gearRatio
    #     self.driveVel = 0
    #     self.turnVel = 0
    #     self.x = 0
    #     self.y = 0
    #     self.Θ = math.pi / 2
    #     self.motorMode = COAST

    #     currRightVal = 0  # current encoder value for right wheel
    #     currLeftVal = 0  # current encoder value for left wheel
    #     currAuxVal = 0  # current encoder value for back wheel

    #     prevRightVal = 0  # previous encoder value for right wheel
    #     prevLeftVal = 0  # previous encoder value for left wheel
    #     prevAuxVal = 0  # previous encoder value for back wheel

    #     self.autoIsRunning = False
    #     self.path = [[]]

    #     wait(100, MSEC)

    #     self.odomThread = Thread(self.updatePosition)

    # # ---------------------------AUTO AND ODOMETRY---------------------------

    # def setAutoPath(self, path):
    #     self.path = path

    # def runAuto(self):
    #     self.set_stopping(COAST)

    #     self.autoIsRunning = True

    #     atPosition = False
    #     for step in self.path:
    #         while not atPosition:
    #             if self.autoIsRunning == False:
    #                 return
    #             atPosition = self.drive_to(step[0], step[1], step[2], step[3],
    #                                        step[4])

    # def stopAuto(self):
    #     self.autoIsRunning = False

    # def updatePosition(self):
    #     while (True):
    #         start = brain.timer.time(MSEC)

    #         prevRightVal = currRightVal
    #         prevLeftVal = currLeftVal
    #         prevAuxVal = currAuxVal

    #         currRightVal = self.rightEncoder.value()
    #         currLeftVal = self.leftEncoder.value()
    #         currAuxVal = self.auxEncoder.value()

    #         dn2 = currRightVal - prevRightVal
    #         dn1 = currLeftVal - prevLeftVal
    #         dn3 = currAuxVal - prevAuxVal

    #         dtheta = self.inchsPerTick * ((dn2 - dn1) / self.L)
    #         dx = self.inchsPerTick * ((dn1 + dn2) / 2.0)
    #         dy = self.inchsPerTick * (dn3 - ((dn2 - dn1) * (self.B / self.L)))

    #         theta = self.Θ + (dtheta / 2.0)
    #         self.x -= -dx * math.cos(-theta) + dy * math.sin(-theta)
    #         self.y += -dx * math.sin(-theta) - dy * math.cos(-theta)
    #         self.Θ += dtheta

    #         while brain.timer.time(MSEC) - start < 10:
    #             continue

    # # ---------------------------DRIVE FUNCTIONS---------------------------

    # def drive_to(self, xTarget: float, yTarget: float, ΘTarget: float,
    #              driveVel: float, turnVel: float):
    #     deltaX, deltaY = self.calcLocalXY(xTarget, yTarget)
    #     deltaTheta = ΘTarget - self.Θ

    #     if abs(deltaX) < 0.25 and abs(deltaY) < 0.25 and abs(
    #             deltaTheta) < 0.035:
    #         print("AT POSITION")
    #         return True

    #     forward = 10 if deltaY > 1 else -10 if deltaY < -1 else 0
    #     strafe = 10 if deltaX > 1 else -10 if deltaX < -1 else 0
    #     rotate = 10 if deltaTheta > 0.045 else -10 if deltaTheta < -0.045 else 0

    #     self.autoDrive(forward, strafe, rotate)

    #     return False

    # def autoDrive(self, forward, strafe, rotate):
    #     self.FL.set_velocity(forward + strafe + rotate, PERCENT)
    #     self.FR.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
    #     self.BR.set_velocity(-forward - strafe + rotate, PERCENT)
    #     self.BL.set_velocity(forward - strafe + rotate, PERCENT)

    #     self.FL.spin_for(FORWARD, 10, MSEC, wait=False)
    #     self.FR.spin_for(FORWARD, 10, MSEC, wait=False)
    #     self.BR.spin_for(FORWARD, 10, MSEC, wait=False)
    #     self.BL.spin_for(FORWARD, 10, MSEC, wait=False)

    # def drive(self, forward, strafe, rotate):
    #     self.FL.set_velocity(forward + strafe + rotate, PERCENT)
    #     self.FR.set_velocity(-1 * (-forward + strafe + rotate), PERCENT)
    #     self.BR.set_velocity(-forward - strafe + rotate, PERCENT)
    #     self.BL.set_velocity(forward - strafe + rotate, PERCENT)

    #     self.FL.spin(FORWARD)
    #     self.FR.spin(FORWARD)
    #     self.BR.spin(FORWARD)
    #     self.BL.spin(FORWARD)

    # def goBackToOG(self):
    #     self.drive_to(0, 0, math.pi / 2, 25, 25)

    # def stop(self):
    #     self.FL.stop()
    #     self.FR.stop()
    #     self.BR.stop()
    #     self.BL.stop()

    # # ---------------------------HELPER FUNCTIONS---------------------------

    # def calcLocalXY(self, xTarget: float, yTarget: float):
    #     deltaX = xTarget - self.x
    #     deltaY = yTarget - self.y

    #     # dist = math.hypot(deltaY, deltaX)
    #     dist = pow(pow(deltaX, 2) + pow(deltaY, 2), 1 / 2)

    #     targetTheta = math.atan2(deltaY, deltaX)
    #     localRelTheta = (targetTheta if targetTheta >= 0 else targetTheta +
    #                      (2 * math.pi)) - self.Θ + (math.pi / 2)

    #     localDeltaX = dist * math.cos(localRelTheta)
    #     localDeltaY = dist * math.sin(localRelTheta)

    #     # limit excessively long and small numbers
    #     localDeltaX = round(localDeltaX, 5)  # 10.0000000000
    #     localDeltaY = round(localDeltaY, 5)  # 10.0000000000

    #     print(localDeltaX, localDeltaY)

    #     return localDeltaX, localDeltaY

    # def set_drive_velocity(self, velocity, units=VelocityUnits.RPM):
    #     self.driveVel = velocity

    # def set_turn_velocity(self, velocity, units=VelocityUnits.RPM):
    #     self.turnVel = velocity

    # def set_stopping(self, mode=BrakeType.COAST):
    #     self.motorMode = mode
    #     self.FL.set_stopping(mode)
    #     self.FR.set_stopping(mode)
    #     self.BR.set_stopping(mode)
    #     self.BL.set_stopping(mode)

    # def resetOdom(self):
    #     self.odomThread.stop()

    #     self.x = 0
    #     self.y = 0
    #     self.Θ = math.pi / 2

    #     # self.odomThread = Thread(self.updatePosition)