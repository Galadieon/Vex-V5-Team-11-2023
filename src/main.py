# ------------------------------------------
#
# 	Project:      Mecanum Robot Program
#	Author:       Lam Ninh & Kaily Do
#	Created:      11/1/2022
#	Description:  VEXcode V5 Python Team 11
#
# ------------------------------------------

# Library imports
from vex import *

import math

# Begin project code
brain = Brain()

# BAD PORT: 5, 6, 7, 8, 11, 12

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# -----------------------------CLASS DECLARATION----------------------------


class Constants:
    """
    ### Constants class - class to hold final constants

    This class holds constants that won't be changed while running.

    #### Arguments:
        None

    #### Returns:
        None

    #### Examples:
        Constants.DRIVETRAIN_FRONT_LEFT\\
        Constants.INDEXER_PORT\\
        Constants.WHEEL_TRAVEL
    """

    FIELDSIZE = 141.0
    TILESIZE = FIELDSIZE / 6.0  # 141 / 6 = 23.5, OG: 24

    TILE___0 = TILESIZE * 0.0
    TILE_0_5 = TILESIZE * 0.5
    TILE___1 = TILESIZE * 1.0
    TILE_1_5 = TILESIZE * 1.5
    TILE___2 = TILESIZE * 2.0
    TILE___3 = TILESIZE * 3.0
    TILE_3_5 = TILESIZE * 3.5
    TILE___4 = TILESIZE * 4.0
    TILE_4_5 = TILESIZE * 4.5
    TILE___5 = TILESIZE * 5.0
    TILE___6 = TILESIZE * 6.0

    LO_SPEED = 50
    HI_SPEED = 100

    ROLLER_OFFSET = 3.0
    TILE_L_R = TILE___0 - ROLLER_OFFSET
    TILE_R_R = TILE___5 + ROLLER_OFFSET

    SHOOT_OFFSET = 4.0
    TILE_L_S = TILE___0 + SHOOT_OFFSET
    TILE_R_S = TILE___5 - SHOOT_OFFSET

    # AutoDrive precision level
    PL_S = 1.0  # 2 inch total
    PL_M = 1.5  # 3 inch total
    PL_L = 2.0  # 4 inch total

    HIGH_GOAL_X = TILESIZE * 0.25  # from center of 1st square
    HIGH_GOAL_Y = TILESIZE * 4.72  # from center of 1st square

    WHEEL_TRAVEL = 4.0 * math.pi
    TRACK_WIDTH = 14.097242
    WHEEL_BASE = 11.5

    # subject to change
    INDEXER_PORT = Ports.PORT2
    FLYWHEEL_PORT1 = Ports.PORT9
    FLYWHEEL_PORT2 = Ports.PORT15
    INTAKE_PORT = Ports.PORT16
    ROLLER_PORT = INTAKE_PORT

    RIGHT_ENCODER = Encoder(brain.three_wire_port.e)
    LEFT_ENCODER = Encoder(brain.three_wire_port.a)
    AUX_ENCODER = Encoder(brain.three_wire_port.c)

    DRIVETRAIN_FRONT_LEFT = Ports.PORT1
    DRIVETRAIN_FRONT_RIGHT = Ports.PORT10
    DRIVETRAIN_BACK_RIGHT = Ports.PORT20
    DRIVETRAIN_BACK_LEFT = Ports.PORT19

    DRIVETRAIN_FORWARD_KP = 8.0
    DRIVETRAIN_FORWARD_KI = 0.05
    DRIVETRAIN_FORWARD_KD = 0.01

    DRIVETRAIN_STRAFE_KP = 8.0
    DRIVETRAIN_STRAFE_KI = 0.05
    DRIVETRAIN_STRAFE_KD = 0.01

    DRIVETRAIN_TURN_KP = 3.0
    DRIVETRAIN_TURN_KI = 0.05
    DRIVETRAIN_TURN_KD = 0.01

    DRIVETRAIN_FINE_CONTROL_VEL = 50

    LEFT_RIGHT_ODOMETRY_DISTANCE = 13.5
    AUX_ODOMETRY_DISTANCE = 5.0
    ODOMETRY_DIAMETER = 2.75
    QUADRATURE_ENCODER_TICKS = 360
    ODOMETRY_CIRCUMFERENCE = math.pi * ODOMETRY_DIAMETER
    INCHES_PER_TICK = ODOMETRY_CIRCUMFERENCE / QUADRATURE_ENCODER_TICKS

    FLYWHEEL_KP = 1
    FLYWHEEL_KI = 0
    FLYWHEEL_KD = 0
    FLYWHEEL_GEAR_RATIO = 84 / 12  # max for motor: 600 RPM, max for flywheel: 4,200 RPM
    SIDE_SHOT = 0.0
    MID_SHOT = 1.0

    INDEXER_GEAR_TEETH = 6
    INDEXER_CHAIN_LINKS = 19


# ---------------------------AUTONOMOUS COMMANDS----------------------------


class AutoFlywheel:
    """
    ### AutoFlywheel class - creates an auto flywheel object

    This class is used to command the flywheel (...) .

    #### Arguments:
        distance : The general distance to high goal
        wait (True) : Determines code execution blocking

    #### Returns:
        A new AutoFlywheel command object

    #### Examples:
        RunCommands(AutoFlywheel(...))
    """

    isRunning = False

    stopAuto = False

    def __init__(self, distance=Constants.HI_SPEED):
        self.distance = distance

        AutoFlywheel.isRunning = False
        AutoFlywheel.stopAuto = False

    def execute(self):
        AutoFlywheel.isRunning = True
        Robot.flywheel.setDistance(self.distance)
        Robot.flywheel.toggleMotor()
        AutoFlywheel.isRunning = False

    def stop(self):
        AutoFlywheel.isRunning = False
        AutoFlywheel.stopAuto = True
        Robot.flywheel.stop()

    def run(self):
        pass


class AutoIntake:
    """
    ### AutoIntake class - creates an auto flywheel object

    This class is used to command the flywheel (...) .

    #### Arguments:
        ...
        wait (True) : Determines code execution blocking

    #### Returns:
        A new AutoIntake command object

    #### Examples:
        RunCommands(AutoIntake(...))
    """

    isRunning = False

    stopAuto = False

    def __init__(self, status=0, wait=True):
        self.wait = wait
        self.status = status
        AutoIntake.isRunning = False
        AutoIntake.stopAuto = False

    def execute(self):
        AutoIntake.isRunning = True
        
        if self.status == 1 : Robot.intake.toggleMotor()
        elif self.status == 0: Robot.intake.stop()

        AutoIntake.isRunning = False

class AutoIndexer:
    """
    ### AutoIndexer class - creates an auto indexer object

    This class is used to command the indexer (...) .

    #### Arguments:
        ...
        wait (True) : Determines code execution blocking

    #### Returns:
        A new AutoIndexer command object

    #### Examples:
        RunCommands(AutoIndexer(...))
    """

    isRunning = False

    stopAuto = False

    def __init__(self, numDisc=3):
        self.numDisc = numDisc

        AutoIndexer.isRunning = False
        AutoIndexer.stopAuto = False

    def execute(self):
        AutoIndexer.isRunning = True

        while self.numDisc > 0:
            if AutoIndexer.stopAuto == True:
                Robot.indexer.stop()
                break
            pushed = Robot.indexer.autoPush()

            if pushed:
                self.numDisc -= 1

        AutoIndexer.isRunning = False

    def stop(self):
        self.isRunning = False
        self.stopAuto = True
        Robot.indexer.stop()


class AutoRoller:
    """
    ### AutoRoller class - creates an auto roller object

    This class is used to command the roller (...) .

    #### Arguments:
        degreesToTurn : Degrees to flip the rollers
        wait (True) : Determines code execution blocking

    #### Returns:
        A new AutoRoller command object

    #### Examples:
        RunCommands(AutoRoller(...))
    """

    isRunning = False

    stopAuto = False

    def __init__(self, degreesToTurn=90, wait=True):
        self.degreesToTurn = degreesToTurn
        self.wait = wait

        AutoRoller.isRunning = False
        AutoRoller.stopAuto = False

    def execute(self):
        """Run the roller to spin how many degrees"""
        AutoRoller.isRunning = True

        Robot.roller.flip(FORWARD, self.degreesToTurn, self.wait)

        AutoRoller.isRunning = False
        AutoRoller.stopAuto = True


class AutoDrive:
    """
    ### AutoDrive class - creates an auto drive object

    This class is used to command the drivetrain to move to a global target.

    #### Arguments:
        xTarget : The global x target
        xTarget : The global y target
        ΘTarget : The global Θ target
        driveVel : The max drive velocity
        turnVel : The max turn velocity
        wait (True) : Determines code execution blocking

    #### Returns:
        A new AutoDrive command object

    #### Examples:
        RunCommands(AutoDrive(0, 10, math.pi / 2, 25, 25), ...)
    """

    isRunning = False

    stopAuto = False

    def __init__(self,
                 xTarget=0.0,
                 yTarget=0.0,
                 ΘTarget=math.pi / 2,
                 driveVel=25.0,
                 turnVel=25.0,
                 overrideAutoClear=False,
                 thresholdX=0.25,
                 thresholdY=0.25,
                 thresholdΘ=math.radians(1),
                 timeOut=5_000,
                 wait=True):
        self.xTarget = xTarget
        self.yTarget = yTarget
        self.ΘTarget = ΘTarget
        self.driveVel = driveVel
        self.turnVel = turnVel
        self.overrideAutoClear = overrideAutoClear
        self.thresholdX = thresholdX
        self.thresholdY = thresholdY
        self.thresholdΘ = thresholdΘ
        self.timeOut = timeOut
        self.wait = wait

        self.thread = None
        self.maintainPos = False

        self.forwardPID = PID(Kp=Constants.DRIVETRAIN_FORWARD_KP,
                              Ki=Constants.DRIVETRAIN_FORWARD_KI,
                              Kd=Constants.DRIVETRAIN_FORWARD_KD)
        self.strafePID = PID(Kp=Constants.DRIVETRAIN_STRAFE_KP,
                             Ki=Constants.DRIVETRAIN_STRAFE_KI,
                             Kd=Constants.DRIVETRAIN_STRAFE_KD)
        self.turnPID = PID(Kp=Constants.DRIVETRAIN_TURN_KP,
                           Ki=Constants.DRIVETRAIN_TURN_KI,
                           Kd=Constants.DRIVETRAIN_TURN_KD)

        AutoDrive.isRunning = False
        AutoDrive.stopAuto = False
        
        Robot.drivetrain.flywheelAsFront(True)

    def execute(self):
        if self.wait:
            self.run()
        else:
            self.thread = Thread(self.run)

    def run(self):
        AutoDrive.isRunning = True
        atTarget = False
        clearedAutoLine = True

        start = brain.timer.time(MSEC)

        while not atTarget:
            if AutoDrive.stopAuto:
                print("STOPPED DRIVE")
                AutoDrive.isRunning = False
                return

            if brain.timer.time(MSEC) - start > self.timeOut:
                RunCommands.stopAll()
                break

            robotX, robotY, robotΘ = Robot.odometry.getPose()

            if self.notClearedAutoLine(robotX,
                                       robotY) or clearedAutoLine is not True:
                xT, yT = self.calcAutoLineClear(robotX, robotY)
                clearedAutoLine = self.driveTo(self.calcLocalXY(xT, yT),
                                               self.ΘTarget, self.driveVel,
                                               self.turnVel)
            else:
                atTarget = self.driveTo(
                    self.calcLocalXY(self.xTarget, self.yTarget), self.ΘTarget,
                    self.driveVel, self.turnVel)

        AutoDrive.isRunning = False

    def driveToOrigin(self):
        self.xTarget = Constants.TILE___1
        self.yTarget = 0
        self.ΘTarget = math.pi / 2

        self.execute()

    def driveTo(self, localXY, ΘTarget: float, driveVel: float,
                turnVel: float):
        '''
        ### AutoDrive
        '''

        self.driveVel = driveVel
        self.turnVel = turnVel

        deltaX, deltaY = localXY
        deltaΘ = ΘTarget - Robot.odometry.Θ

        if abs(deltaX) <= self.thresholdX and abs(
                deltaY) <= self.thresholdY and abs(
                    deltaΘ) <= self.thresholdΘ and self.maintainPos == False:
            return True

        forward, strafe, turn = self.updatePID(deltaX, deltaY,
                                               math.degrees(deltaΘ))

        Robot.drivetrain.drive(forward, strafe, -turn)

        wait(10, MSEC)

        return False

    def updatePID(self, deltaX, deltaY, deltaΘ):
        forward = self.forwardPID.update(deltaY, 0.0)
        strafe = self.strafePID.update(deltaX, 0.0)
        turn = self.turnPID.update(deltaΘ, 0.0)

        # limits max speed, everything else same
        if abs(forward) > self.driveVel:
            forward = (forward / abs(forward)) * self.driveVel

        if abs(strafe) > self.driveVel:
            strafe = (strafe / abs(strafe)) * self.driveVel

        if abs(turn) > self.turnVel:
            turn = (turn / abs(turn)) * self.turnVel

        return forward, strafe, turn

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

        localDeltaX = round(localDeltaX, 7)  # 10.0000000
        # limit excessively long and small numbers
        localDeltaY = round(localDeltaY, 7)  # 10.0000000

        return localDeltaX, localDeltaY

    def notClearedAutoLine(self, x, y):
        return False if self.overrideAutoClear == True else y > self.calcAutoLineY(
            x)

    def calcAutoLineClear(self, robotX, robotY):
        xTarget = ((robotX + robotY + Constants.TILE___1 + 19.8) / 2.0) + 2
        yTarget = self.calcAutoLineY(xTarget) - 2
        return xTarget, yTarget

    def calcAutoLineY(self, x):
        return x - 19.8


class AutoAlignShoot(AutoDrive):

    isRunning = False

    stopAuto = False

    autoFlywheel = None

    autoIndexer = None

    def __init__(self,
                 xTarget=0.0,
                 yTarget=0.0,
                 ΘTarget=math.pi / 2,
                 distance=Constants.HI_SPEED,
                 driveVel=25.0,
                 turnVel=25.0,
                 overrideAutoClear=False,
                 thresholdX=0.25,
                 thresholdY=0.25,
                 thresholdΘ=math.radians(1),
                 timeOut=5_000,
                 wait=True,
                 discs=3):
        robotX, robotY, robotΘ = Robot.odometry.getPose()
        super().__init__(xTarget, yTarget, self.calcAngleToHi(robotX, robotY),
                         driveVel, turnVel, overrideAutoClear, thresholdX,
                         thresholdY, thresholdΘ, timeOut, wait)

        self.distance = distance
        self.discs = discs

        AutoAlignShoot.isRunning = False
        AutoAlignShoot.stopAuto = False

        AutoAlignShoot.autoFlywheel = None
        AutoAlignShoot.autoIndexer = None

    # may need to fix this later
    def calcAngleToHi(self, robotX, robotY):
        return math.atan2(Constants.HIGH_GOAL_Y - robotY,
                          Constants.HIGH_GOAL_X - robotX)

    def execute(self):
        AutoAlignShoot.isRunning = True

        AutoAlignShoot.autoFlywheel = AutoFlywheel(distance=self.distance)
        AutoAlignShoot.autoFlywheel.execute()

        self.alignMaintainPos()

        AutoAlignShoot.autoIndexer = AutoIndexer(self.discs)
        AutoAlignShoot.autoIndexer.execute()

        AutoAlignShoot.stopAll()

    @staticmethod
    def stopAll():
        AutoAlignShoot.isRunning = False
        AutoAlignShoot.stopAuto = True
        # AutoDrive.stopAuto = True
        if AutoAlignShoot.autoFlywheel != None:
            AutoAlignShoot.autoFlywheel.stop()
        # if AutoAlignShoot.autoIndexer != None:
        #     AutoAlignShoot.autoIndexer.stop()

    def alignMaintainPos(self):
        print("ATTEMPTING ALIGNMENT ...\n")
        super().execute()
        # print("ALIGNMENT COMPLETED\nCOMMENCING LAUNCHES\n")
        # self.maintainPos = True
        # self.wait = False
        # super().execute()


# ---------------------------AUTONOMOUS ROUTINES----------------------------


class RunCommands:
    stopAuto = False
    pauseAuto = False
    isRunning = False

    def __init__(self, *commandList):
        RunCommands.stopAuto = False
        RunCommands.pauseAuto = False
        RunCommands.isRunning = True

        Robot.drivetrain.set_stopping(HOLD)

        for command in commandList:
            while RunCommands.pauseAuto:
                continue
            if RunCommands.stopAuto:
                break

            command.execute()

        Robot.drivetrain.set_stopping(COAST)

        RunCommands.isRunning = False

    @staticmethod
    def stopAll():
        RunCommands.isRunning = False
        RunCommands.stopAuto = True

        AutoDrive.stopAuto = True
        AutoAlignShoot.stopAll()
        AutoFlywheel.stopAuto = True
        AutoIndexer.stopAuto = True
        AutoIntake.stopAuto = True
        AutoRoller.stopAuto = True

    @staticmethod
    def pause():
        RunCommands.pauseAuto = True

    @staticmethod
    def unpause():
        RunCommands.pauseAuto = False


class TestMode:

    def __init__(self):
        Robot.odometry.setPose(Constants.TILE___1, Constants.TILE___0,
                               math.pi / 2)
        commandRun = RunCommands(
            AutoDrive(Constants.TILE___1, Constants.TILE___1, math.pi / 2, 100,
                      100, True),
            AutoDrive(Constants.TILE___2, Constants.TILE___1, math.pi / 2, 100,
                      100, True),
            AutoDrive(Constants.TILE___1, Constants.TILE___0, math.pi / 2, 100,
                      100, True),
            AutoDrive(Constants.TILE___1, Constants.TILE___2, 0, 70, 100,
                      True),
            AutoDrive(Constants.TILE___3, Constants.TILE___2, math.pi / 2, 70,
                      100, True),
            AutoDrive(Constants.TILE___1, Constants.TILE___0,
                      (3 * math.pi) / 2, 70, 100, True),
        )


class LeftAuto1:

    def __init__(self):
        Robot.odometry.setPose(Constants.TILE___1, Constants.TILE___0,
                               math.pi / 2)
        commandRun = RunCommands(
            AutoDrive(Constants.TILE___1, Constants.TILE_L_R, math.pi / 2, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE___1,
                           Constants.TILE_L_S,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),

            # intake on
            AutoDrive(Constants.TILE___3, Constants.TILE___2,
                      (5 * math.pi) / 4, 70, 100, True),
            AutoAlignShoot(Constants.TILE___3,
                           Constants.TILE___2,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),
            AutoDrive(Constants.TILE_4_5, Constants.TILE_3_5,
                      (5 * math.pi) / 4, 70, 100, True),
            AutoDrive(Constants.TILE___5, Constants.TILE___4, math.pi, 100,
                      100, True),

            # intake off
            AutoDrive(Constants.TILE_R_R, Constants.TILE___4, math.pi, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE_R_S,
                           Constants.TILE___4,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),
        )


class RightAuto1:

    def __init__(self):
        Robot.odometry.setPose(Constants.TILE___5, Constants.TILE___3, math.pi)
        commandRun = RunCommands(
            AutoDrive(Constants.TILE___5, Constants.TILE___4, math.pi, 100,
                      100, True),
            AutoDrive(Constants.TILE_R_R, Constants.TILE___4, math.pi, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE_R_S,
                           Constants.TILE___4,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),
            
            AutoIntake(status=1),
            AutoDrive(Constants.TILE___3, Constants.TILE___2, math.pi / 4, 70,
                      100),
            AutoAlignShoot(Constants.TILE___3,
                           Constants.TILE___2,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),
            AutoDrive(Constants.TILE_1_5, Constants.TILE_0_5, math.pi / 4, 70,
                      100),
            AutoDrive(Constants.TILE___1, Constants.TILE___0, math.pi / 2, 100,
                      100, True),

            AutoIntake(status=0),
            AutoDrive(Constants.TILE___1, Constants.TILE_L_R, math.pi / 2, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE___1,
                           Constants.TILE_L_S,
                           0,
                           Constants.HI_SPEED,
                           100,
                           100,
                           True,
                           timeOut=3_000),
        )


# -------------------------------UTILITIES-------------------------------


class PID:
    """
    ### PID class - create PID controller

    This class is used to simplify creation of PID controllers.

    #### Arguments:
        Kp (optional) : The proportional constant (how reactive the PID controller is)
        Ki (optional) : The integral constant (how impactful the PID controllelr is for small movements)
        Ki (optional) : The integral constant (how much to dampen the PID controller)

    #### Returns:
        A new PID controller object

    #### Examples:
        pid1 = PID(1)\\
        pid2 = PID(1, 0.5)\\
        pid2 = PID(1, 0.5, 0.25)
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previousError = 0.0
        self.integral = 0.0

    def update(self, target, current):
        error = target - current

        self.integral += error
        if abs(error) < 0.5 or abs(error) > 5.0: self.integral = 0.0

        derivative = error - self.previousError
        self.previousError = error

        controlledValue = (self.Kp * error) + (self.Ki * self.integral) + (
            self.Kd * derivative)

        if controlledValue > 100: controlledValue = 100
        if controlledValue < -100: controlledValue = -100

        return controlledValue


class MyController:
    """
    ### MyController class - holds all controller features

    Allows controller use in a separate class.

    #### Arguments:
        None

    #### Returns:
        A new MyController object.

    #### Examples:
        myController1 = MyController()
    """

    def __init__(self):
        self.controllerEnabled = True
        self.controller = Controller(PRIMARY)

        self.manualIndexer = True

        self.registerEventHandlers()

        Thread(self.controllerLoop)

    def controllerLoop(self):
        deadZoneVal = 5  # PERCENT
        while (True):
            if self.controllerEnabled:
                forward = self.axisCurve(self.controller.axis3.position())
                strafe = self.axisCurve(self.controller.axis4.position())
                turn = self.axisCurve(self.controller.axis1.position())

                if abs(forward) > deadZoneVal or abs(
                        strafe) > deadZoneVal or abs(turn) > deadZoneVal:
                    Robot.drivetrain.drive(
                        forward * (Robot.drivetrain.driveVel / 100),
                        strafe * (Robot.drivetrain.driveVel / 100),
                        turn * (Robot.drivetrain.turnVel / 100))
                else:
                    Robot.drivetrain.stop()

            wait(10, MSEC)

    def axisCurve(self, x):
        # return (pow(x, 3)) / 10_000
        return x

    def updateRow1(self):
        # X: _ Y: _ Θ: _
        self.controller.screen.clear_row(1)

        self.controller.screen.set_cursor(1, 1)

        robotX, robotY, robotΘ = Robot.odometry.getPose()

        self.controller.screen.print(robotX, robotY, math.degrees(robotΘ))

    def updateRow2(self):
        # D: 24 V: 4,200
        self.controller.screen.clear_row(2)

        self.controller.screen.set_cursor(2, 1)

        self.controller.screen.print("FWD:", Robot.flywheel.distance, "FWV:",
                                     Robot.flywheel.flywheelVel)

    def updateRow3(self):
        # FC: False MI: False
        self.controller.screen.clear_row(3)

        self.controller.screen.set_cursor(3, 1)

        fineControl = Robot.drivetrain.driveVel == Constants.DRIVETRAIN_FINE_CONTROL_VEL

        self.controller.screen.print("FC:", fineControl, "MI:",
                                     self.manualIndexer)

    def registerEventHandlers(self):
        self.controller.buttonL1.pressed(self.L1_Pressed)
        self.controller.buttonL2.pressed(self.L2_Pressed)
        self.controller.buttonR1.pressed(self.R1_Pressed)
        self.controller.buttonR2.pressed(self.R2_Pressed)

        self.controller.buttonA.pressed(self.A_Pressed)
        self.controller.buttonB.pressed(self.B_Pressed)
        self.controller.buttonX.pressed(self.X_Pressed)
        self.controller.buttonY.pressed(self.Y_Pressed)

        self.controller.buttonUp.pressed(self.Up_Pressed)
        self.controller.buttonDown.pressed(self.Down_Pressed)
        self.controller.buttonLeft.pressed(self.Left_Pressed)
        self.controller.buttonRight.pressed(self.Right_Pressed)

    # ---------------------------BUTTON FUNCTIONS---------------------------
    """
    #### Available button commands in use
    
    self.changeDriveTrainVel()
    Robot.odometry.resetPose()
    self.toggleAuto()
    self.toggleDriveTrainMode()
    AutoDrive().driveToOrigin()
    """

    #  /L2 | R2\
    # / L1 | R1 \
    # |---------|
    # \_________/

    def L1_Pressed(self):
        if Robot.drivetrain.getFrontIsFlywheel():
            Robot.flywheel.toggleMotor()
        else:
            Robot.intake.reverseMotor()

    def L2_Pressed(self):
        Robot.drivetrain.changeFront()

    def R1_Pressed(self):
        if Robot.drivetrain.getFrontIsFlywheel():
            Robot.flywheel.toggleSpeed()
        else:
            Robot.roller.flip()

    def R2_Pressed(self):
        if Robot.drivetrain.getFrontIsFlywheel():
            if self.toggleManualIndexer() == False:
                if self.manualIndexer:
                    Robot.indexer.push()
                else:
                    # Robot.indexer.autoPush()
                    pass
        else:
            Robot.intake.toggleMotor()

    """
      X
    Y   A
      B
    """

    def X_Pressed(self):
        # self.printFWVelDict()
        pass

    def A_Pressed(self):
        Robot.flywheel.increaseVelocity()

    def B_Pressed(self):
        Robot.flywheel.decreaseVelocity()

    def Y_Pressed(self):
        pass

    """
      ↑
    ←   →
      ↓
    """

    def Up_Pressed(self):
        pass

    def Right_Pressed(self):
        self.changeDriveTrainVel()

    def Down_Pressed(self):
        # Robot.odometry.reset()
        pass

    def Left_Pressed(self):
        # self.toggleDriveTrainMode()
        pass

    # ----------------------BUTTON HELPER METHODS------------------------

    def toggleRobotMode(self):
        if self.mode == 0:
            self.mode = 1
        else:
            self.mode = 0

    def printFWVelDict(self):
        print(Robot.flywheel.velocityDict)

    def changeDriveTrainVel(self):
        if Robot.drivetrain.driveVel == 100:
            Robot.drivetrain.set_turn_velocity(
                Constants.DRIVETRAIN_FINE_CONTROL_VEL)
            Robot.drivetrain.set_drive_velocity(
                Constants.DRIVETRAIN_FINE_CONTROL_VEL)
        elif Robot.drivetrain.driveVel == Constants.DRIVETRAIN_FINE_CONTROL_VEL:
            Robot.drivetrain.set_turn_velocity(100)
            Robot.drivetrain.set_drive_velocity(100)
        self.updateRow3()

    # def toggleDriveTrainMode(self):
    #     if Robot.drivetrain.getMotorMode() == BRAKE:
    #         Robot.drivetrain.set_stopping(COAST)
    #     elif Robot.drivetrain.getMotorMode() == COAST:
    #         Robot.drivetrain.set_stopping(BRAKE)

    def toggleManualIndexer(self):
        start = brain.timer.time(MSEC)

        while self.controller.buttonR2.pressing():
            if brain.timer.time(MSEC) - start > 1_000:
                self.manualIndexer = not self.manualIndexer
                self.updateRow3()
                return True
            wait(10, MSEC)

        return False


class Odometry:
    """
    ### Odometry class - creates odometry object

    This class is used to hold information about robot pose and quadrature encoders.

    #### Arguments:
        rightEncoder : The encoder on the right of robot
        leftEncoder : The encoder on the left of robot
        auxEncoder : The encoder on the auxillary ("back") side of robot

    #### Returns:
        A new Odometry object.

    #### Examples:
        odometry1 = Odometry(Constants.RIGHT_ENCODER,\\
        Constants.LEFT_ENCODER,\\
        Constants.AUX_ENCODER)
    """

    def __init__(self, rightEncoder, leftEncoder, auxEncoder):
        self.x = Constants.TILE___1
        self.y = 0
        self.Θ = math.pi / 2

        self.rightEncoder = rightEncoder
        self.leftEncoder = leftEncoder
        self.auxEncoder = auxEncoder

        self.threadIsRunning = False
        self.threadIsPaused = False

        self.resetOdomEncoders = False
        self.resetOdomPose = False

        # Thread(self.updatePose)

        self.inchsPerTick = Constants.INCHES_PER_TICK
        self.LR_Distance = Constants.LEFT_RIGHT_ODOMETRY_DISTANCE
        self.B_Distance = Constants.AUX_ODOMETRY_DISTANCE
        
        self.currRightVal = 0  # current encoder value for right wheel
        self.currLeftVal = 0  # current encoder value for left wheel
        self.currAuxVal = 0  # current encoder value for back wheel
        
        self.prevRightVal = 0  # previous encoder value for right wheel
        self.prevLeftVal = 0  # previous encoder value for left wheel
        self.prevAuxVal = 0  # previous encoder value for back whee

    def updatePose(self):
        screenStartTime = brain.timer.time(MSEC)
        screenUpdateInterval = 100

        self.threadIsRunning = True

        # while (self.threadIsRunning):
        if self.Θ >= 360.0: self.Θ = 0.0
        if self.Θ < 0.0: self.Θ = 360.0
        
        # anytime that x or y robot values are greater than 1,000 inches, reset encoders & pose
        if abs(self.x) > 1_000 or abs(self.y) > 1_000 or abs(self.Θ) > 1_000:
            self.resetPose()
            self.resetEncoders()

        if self.resetOdomPose == True:
            self.resetPose()

        if self.resetOdomEncoders == True:
            self.resetEncoders()

        if self.threadIsPaused:
            wait(1, MSEC)
            pass

        start = brain.timer.time(MSEC)

        self.prevRightVal = self.currRightVal
        self.prevLeftVal = self.currLeftVal
        self.prevAuxVal = self.currAuxVal

        self.currRightVal = self.rightEncoder.value()
        self.currLeftVal = self.leftEncoder.value()
        self.currAuxVal = self.auxEncoder.value()

        dn2 = self.currRightVal - self.prevRightVal
        dn1 = self.currLeftVal - self.prevLeftVal
        dn3 = self.currAuxVal - self.prevAuxVal

        dtheta = self.inchsPerTick * ((dn2 - dn1) / self.LR_Distance)
        dx = self.inchsPerTick * ((dn1 + dn2) / 2.0)
        dy = self.inchsPerTick * (dn3 - ((dn2 - dn1) *
                                    (self.B_Distance / self.LR_Distance)))

        theta = self.Θ + (dtheta / 2.0)
        self.x += -dx * math.cos(-theta) + dy * math.sin(-theta)
        self.y -= -dx * math.sin(-theta) - dy * math.cos(-theta)
        self.Θ += dtheta

        screenEndTime = brain.timer.time(MSEC)

        if screenEndTime > screenStartTime + screenUpdateInterval:
            screenStartTime = screenEndTime
            # myController.updateRow1()

        while brain.timer.time(MSEC) - start < 7.5:
            continue

    def stop(self):
        self.threadIsRunning = False

    def getPose(self):
        return self.x, self.y, self.Θ

    def reset(self):
        self.resetOdomPose = True
        self.resetOdomEncoders = True

    def resetPose(self):
        self.setPose(Constants.TILE___1, 0, math.pi / 2)
        self.resetOdomPose = False

    def setPose(self, newX, newY, newΘ):
        self.x = newX
        self.y = newY
        self.Θ = newΘ

    def resetEncoders(self):
        self.rightEncoder.set_position(0, DEGREES)
        self.leftEncoder.set_position(0, DEGREES)
        self.auxEncoder.set_position(0, DEGREES)
        self.resetOdomEncoders = False


# -------------------------------SUBSYSTEMS------------------------------


class MecanumDriveTrain:
    """
    ### MecanumDrivetrain class - creates mecanum drivetrain

    This class is used to create and use mecanum drivetrain.

    #### Arguments:
        FL : The front left motor
        FR : The front right motor
        BR : The back right motor
        BL : The back left motor

    #### Returns:
        A new MecanumDriveTrain object.

    #### Examples:
        drivetrain1 = MecanumDriveTrain(Constants.DRIVETRAIN_FRONT_LEFT,\\
        Constants.DRIVETRAIN_FRONT_RIGHT,\\
        Constants.DRIVETRAIN_BACK_RIGHT,\\
        Constants.DRIVETRAIN_BACK_LEFT)
    """

    def __init__(self, FL, FR, BR, BL):
        self.motorFrontLeft = Motor(FL, GearSetting.RATIO_18_1, False)
        self.motorFrontRight = Motor(FR, GearSetting.RATIO_18_1, True)
        self.motorBackRight = Motor(BR, GearSetting.RATIO_18_1, True)
        self.motorBackLeft = Motor(BL, GearSetting.RATIO_18_1, False)

        self.motorFrontLeft.set_max_torque(100, PERCENT)
        self.motorFrontRight.set_max_torque(100, PERCENT)
        self.motorBackRight.set_max_torque(100, PERCENT)
        self.motorBackLeft.set_max_torque(100, PERCENT)

        self.driveVel = 100
        self.turnVel = 100
        self.motorMode = COAST
        self.flywheelIsFront = True
    
    def changeFront(self):
        self.flywheelIsFront = not self.flywheelIsFront
    
    def flywheelAsFront(self, bool):
        self.flywheelIsFront = bool

    def drive(self, forward, strafe, turn):
        if self.flywheelIsFront:
            forward = -forward
            strafe = -strafe
            turn = -turn
            
        self.motorFrontLeft.set_velocity(forward + strafe + turn, PERCENT)
        self.motorFrontRight.set_velocity(forward - strafe - turn, PERCENT)
        self.motorBackRight.set_velocity(forward + strafe - turn, PERCENT)
        self.motorBackLeft.set_velocity(forward - strafe + turn, PERCENT)

        self.motorFrontLeft.spin(FORWARD)
        self.motorFrontRight.spin(FORWARD)
        self.motorBackRight.spin(FORWARD)
        self.motorBackLeft.spin(FORWARD)

    def stop(self):
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackRight.stop()
        self.motorBackLeft.stop()

    def set_drive_velocity(self, velocity):
        """ #### Assume Percent """
        self.driveVel = velocity

    def set_turn_velocity(self, velocity):
        """ #### Assume Percent """
        self.turnVel = velocity

    def set_stopping(self, mode=BrakeType.COAST):
        self.motorMode = mode
        self.motorFrontLeft.set_stopping(mode)
        self.motorFrontRight.set_stopping(mode)
        self.motorBackRight.set_stopping(mode)
        self.motorBackLeft.set_stopping(mode)

    def getMotorMode(self):
        return self.motorMode
    
    def getFrontIsFlywheel(self):
        return self.flywheelIsFront


class Flywheel:
    """
    ### Flywheel class - creates flywheel object

    This class is to create and run robot flywheel that launches discs.

    #### Arguments:
        One or more Motor class instances (Motor objects)

    #### Returns:
        A new Flywheel object.

    #### Examples:
        flywheel1 = Flywheel(Constants.FLYWHEEL_PORT1)\\
        flywheel2 = Flywheel(Constants.FLYWHEEL_PORT1, Constants.FLYWHEEL_PORT2)
    """

    def __init__(self, *motors):
        # self.motorGroup = MotorGroup(*[motors])
        self.motorGroup = Motor(motors[0], GearSetting.RATIO_6_1, False)

        # self.flywheelPID = PID(Kp=1)
        self.endgameLaunched = False
        self.flywheelVel = 1_400
        self.motorVel = self.calcMotorVel(self.flywheelVel)

        self.isRunning = False

        self.distance = Constants.LO_SPEED

        # for 84 : 12 max: 4_200 RPM (Our robot's max)
        # for 84 : 36 max: 1_400 RPM

        self.velocityDict = {
            # need empirical data & verification

            # Constants.MID_SHOT:
            # 4_200 / 2.0 + 4_200 / 8.0,  # 2,625
            # Constants.SIDE_SHOT:
            # 4_200 * (2.0 / 3.0) + 4_200 / 8.0,  # 3,325

            Constants.LO_SPEED:
            4_200 * (1.0 / 2.0),  # 2,100
            Constants.HI_SPEED:
            4_200 * (4.0 / 4.0)  # 4,200
        }

        self.motorGroup.set_max_torque(100, PERCENT)

    def stop(self):
        self.isRunning = False
        self.motorGroup.stop()

    def calcMotorVel(self, flywheelVel):
        return flywheelVel / Constants.FLYWHEEL_GEAR_RATIO  # 1,400 / 7 = 200 RPM

    def toggleMotor(self):
        if self.isRunning == True:
            self.isRunning = False
            print("MOTOR STOPPED")
            self.motorGroup.stop()
        else:
            self.isRunning = True
            self.motorGroup.spin(FORWARD, self.motorVel, RPM)
            print("MOTOR SPINNING")

    def isAtSetVel(self):
        currMotorVel = self.motorGroup.velocity(RPM)

        if self.motorVel - 5 <= currMotorVel or currMotorVel <= self.motorVel + 5:
            return True
        return False

    def toggleSpeed(self):
        # if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
        #     self.distance = Constants.LO_SPEED
        # else:
        if self.distance == Constants.LO_SPEED:
            self.distannce = Constants.HI_SPEED
        else:
            self.distance = Constants.LO_SPEED

        self.updateVel()

    def increaseVelocity(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.LO_SPEED
        else:
            self.velocityDict[self.distance] += 50

            if self.velocityDict[self.distance] > 4_200.0:
                self.velocityDict[self.distance] = 4_200.0

        self.updateVel()

    def decreaseVelocity(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.LO_SPEED
        else:
            self.velocityDict[self.distance] -= 50

            if self.velocityDict[self.distance] < 0.0:
                self.velocityDict[self.distance] = 0.0

        self.updateVel()

    def updateVel(self):
        if self.distance in self.velocityDict.keys():
            myController.updateRow2()
            self.setVelocity(self.velocityDict[self.distance])
            print(self.__class__.__name__, self.velocityDict[self.distance])

    def setDistance(self, distance):
        self.distance = distance
        self.updateVel()
    
    def setVelocity(self, flywheelVel):
        self.flywheelVel = flywheelVel
        self.motorVel = self.calcMotorVel(self.flywheelVel)
        self.motorGroup.set_velocity(self.motorVel, RPM)
        if self.isRunning:
            self.motorGroup.spin(FORWARD, self.motorVel, RPM)


class Indexer:
    """
    ### Indexer class - creates indexer object

    This class is to create and run robot indexer to push discs into flywheel.

    #### Arguments:
        motor : The indexer motor to push discs

    #### Returns:
        A new Indexer object.

    #### Examples:
        indexer1 = Indexer(Constants.INDEXER_PORT)
    """

    def __init__(self, motor):
        self.motor = Motor(motor, GearSetting.RATIO_18_1, True)
        self.motor.set_stopping(HOLD)
        self.motor.set_velocity(100, PERCENT)
        self.isRunning = False

        # calculate the degrees the motor turns to move 1 chain forward
        self.degreesPerTeeth = 360 / Constants.INDEXER_GEAR_TEETH

        # calculate the total degrees the motor turns to complete 1 cycle
        self.degreesPerCycle = Constants.INDEXER_CHAIN_LINKS * self.degreesPerTeeth

        self.motor.set_max_torque(25, PERCENT)

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        if self.motor.is_spinning():
            self.stop()
        else:
            self.push()

    def autoPush(self):
        if Robot.flywheel.isAtSetVel() and self.motor.is_done():
            self.push()
            return True
        return False

    def push(self):
        if not self.motor.is_spinning() and Robot.flywheel.isRunning:
            self.motor.spin_for(FORWARD,
                                self.degreesPerCycle,
                                DEGREES,
                                wait=True)

    def stop(self):
        self.motor.stop()


class Intake:
    """
    ### Flywheel class - creates flywheel object

    This class is to create and run robot flywheel.

    #### Arguments:
        motor : The intake motor to collect or remove jammed discs

    #### Returns:
        A new Intake object.

    #### Examples:
        intake1 = Intake(Constants.INTAKE_PORT)
    """

    def __init__(self, motor):
        self.motor = Motor(motor, GearSetting.RATIO_18_1, False)
        self.motor.set_max_torque(100, PERCENT)
        self.isRunning = False

    def toggleMotor(self, direction=FORWARD):
        if self.motor.is_spinning() or self.isRunning:
            self.stop()
        else:
            self.run()

    def run(self):
        self.motor.spin(FORWARD, 100, PERCENT)
        self.isRunning = True
    
    def stop(self):
        self.motor.stop()
        self.isRunning = False

    def reverseMotor(self):
        self.motor.spin_for(REVERSE, 360 * 2, DEGREES, 100, PERCENT)
        self.isRunning = False


class Roller:
    """
    ### Flywheel class - creates flywheel object
    This class is to create and run robot flywheel.
    #### Arguments:
        motor : The intake motor to collect or remove jammed discs
    #### Returns:
        A new Intake object.
    #### Examples:
        intake1 = Intake(Constants.INTAKE_PORT)
    """

    def __init__(self, motor, wait=True):
        self.motor = Motor(motor, GearSetting.RATIO_18_1, False)
        self.wait = wait

        self.motor.set_max_torque(25, PERCENT)

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        pass

    def flip(self, direction=FORWARD, degreesToTurn=90, wait=False):
        self.motor.spin_for(direction, degreesToTurn, DEGREES, 50, PERCENT,
                            wait)


# ---------------------------------ROBOT--------------------------------


class Robot:
    """
    ### Robot class - creates robot object

    This class is to create and run robot subsystems and autonomous from a static context.

    #### Arguments:
        None

    #### Returns:
        A new Robot object (unnecessary if accessing subsystem functions).

    #### Examples:
        robot1 = Robot() # unnecessary
    """

    # Utility variable instantiation and initialization
    odometry = Odometry(Constants.RIGHT_ENCODER, Constants.LEFT_ENCODER,
                        Constants.AUX_ENCODER)

    # Subsystem variable instantiation and initialization
    drivetrain = MecanumDriveTrain(Constants.DRIVETRAIN_FRONT_LEFT,
                                   Constants.DRIVETRAIN_FRONT_RIGHT,
                                   Constants.DRIVETRAIN_BACK_RIGHT,
                                   Constants.DRIVETRAIN_BACK_LEFT)

    indexer = Indexer(Constants.INDEXER_PORT)

    flywheel = Flywheel(Constants.FLYWHEEL_PORT1)

    intake = Intake(Constants.INTAKE_PORT)

    roller = Roller(Constants.ROLLER_PORT)


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS


def non_competition_start():
    Driver_Control()


def Default_Motor_Speed():
    Robot.drivetrain.set_drive_velocity(100)
    Robot.drivetrain.set_turn_velocity(100)


# AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS


def vexcode_auton_function():
    auton_task_0 = Thread(Autonomous_Control)
    while (competition.is_autonomous() and competition.is_enabled()):
        Robot.odometry.updatePose()
        wait(10, MSEC)
    auton_task_0.stop()


def Autonomous_Control():
    brain.screen.print("Starting auto")
    RightAuto1()


# DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS


def vexcode_driver_function():
    driver_control_task_0 = Thread(Driver_Control)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    Robot.drivetrain.stop()
    driver_control_task_0.stop()


def Driver_Control():
    Default_Motor_Speed()
    Robot.flywheel.setDistance(Constants.TILE___1)


# ---------------------------REQUIRED CODE---------------------------

# wait for rotation sensor to fully initialize
wait(30, MSEC)

myController = MyController()

competition = Competition(vexcode_driver_function, vexcode_auton_function)

# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

non_competition_start()
