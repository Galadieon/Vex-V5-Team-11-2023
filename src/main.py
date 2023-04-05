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

    ROLLER_OFFSET = 3.0
    TILE_L_R = TILE___0 - ROLLER_OFFSET
    TILE_R_R = TILE___5 + ROLLER_OFFSET

    SHOOT_OFFSET = 4.0
    TILE_L_S = TILE___0 + SHOOT_OFFSET
    TILE_R_S = TILE___5 - SHOOT_OFFSET

    # AutoDrive precision level
    PL_S = 1.0 # 2 inch total
    PL_M = 1.5 # 3 inch total
    PL_L = 2.0 # 4 inch total

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

    FLYWHEEL_KP = 0.05
    FLYWHEEL_KI = 0.0
    FLYWHEEL_KD = 0.0
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

    def __init__(self, distance=Constants.SIDE_SHOT):
        self.distance = distance

        AutoFlywheel.isRunning = False
        AutoFlywheel.thread = Thread(self.run)
        AutoFlywheel.thread.stop()

        print(self.__class__.__name__, "Initialized")

    def execute(self):
        self.run()

    def run(self):
        AutoFlywheel.isRunning = True
        Robot.flywheel.setDistance(self.distance)
        Robot.flywheel.toggleMotor()
        AutoFlywheel.isRunning = False

    @staticmethod
    def stop():
        if AutoFlywheel.isRunning:
            AutoFlywheel.isRunning = False
            AutoFlywheel.thread.stop()
            wait(50, MSEC)
            Robot.flywheel.stop()


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

    def __init__(self, wait=True):
        # TODO: add initialization code to run the first time object is created
        self.wait = wait
        AutoIntake.isRunning = False
        AutoIntake.thread = Thread(self.run)
        AutoIntake.thread.stop()

        print(self.__class__.__name__, "Initialized")

    # TODO: add any other helper methods

    def execute(self):
        self.run()
    
    def run(self):
        AutoIntake.isRunning = True

        # TODO: add code to run intake when command is executed

        AutoIntake.isRunning = False

    @staticmethod
    def stop():
        if AutoIntake.isRunning:
            AutoIntake.isRunning = False
            AutoIntake.thread.stop()
            wait(50, MSEC)
            Robot.intake.stop()

        # TODO: add code to stop the physical flywheel


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

    def __init__(self, numDisc=3):
        self.numDisc = numDisc

        AutoIndexer.isRunning = False
        AutoIndexer.thread = Thread(self.run)
        AutoIndexer.thread.stop()

        print(self.__class__.__name__, "Initialized")

    def execute(self):
        self.run()
    
    def run(self):
        AutoIndexer.isRunning = True

        while self.numDisc > 0:
            pushed = Robot.indexer.autoPush()

            if pushed:
                self.numDisc -= 1

        AutoIndexer.isRunning = False

    @staticmethod
    def stop():
        if AutoIndexer.isRunning:
            AutoIndexer.isRunning = False
            AutoIndexer.thread.stop()
            wait(50, MSEC)
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

    def __init__(self, degreesToTurn=90, wait=True):
        self.degreesToTurn = degreesToTurn
        self.wait = wait

        AutoRoller.isRunning = False
        AutoRoller.thread = Thread(self.run)
        AutoRoller.thread.stop()

        print(self.__class__.__name__, "Initialized")

    def execute(self):
        self.run()
    
    def run(self):
        AutoRoller.isRunning = True

        Robot.roller.flip(FORWARD, self.degreesToTurn, self.wait)

        AutoRoller.isRunning = False
    
    @staticmethod
    def stop():
        if AutoRoller.isRunning:
            AutoRoller.isRunning = False
            AutoRoller.thread.stop()
            wait(50, MSEC)
            Robot.roller.stop()


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
        AutoDrive.thread = Thread(self.run)
        AutoDrive.thread.stop()
        
        print(self.__class__.__name__, "Initialized")

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
    
    @staticmethod
    def stop():
        if AutoDrive.isRunning:
            AutoDrive.isRunning = False
            AutoDrive.thread.stop()
            wait(50, MSEC)
            Robot.drivetrain.stop()

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
        forward = self.forwardPID.update(deltaY, 0.0, PERCENT)
        strafe = self.strafePID.update(deltaX, 0.0, PERCENT)
        turn = self.turnPID.update(deltaΘ, 0.0, PERCENT)

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
                 distance=Constants.SIDE_SHOT,
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
        AutoAlignShoot.thread = Thread(self.run)
        AutoAlignShoot.thread.stop()

        AutoAlignShoot.autoFlywheel = None
        AutoAlignShoot.autoIndexer = None
        
        print(self.__class__.__name__, "Initialized")

    # may need to fix this later
    def calcAngleToHi(self, robotX, robotY):
        return math.atan2(Constants.HIGH_GOAL_Y - robotY,
                          Constants.HIGH_GOAL_X - robotX)

    def execute(self):
        self.run()
    
    def run(self):
        AutoAlignShoot.isRunning = True

        AutoAlignShoot.autoFlywheel = AutoFlywheel(distance=self.distance)
        AutoAlignShoot.autoFlywheel.execute()

        self.alignMaintainPos()

        AutoAlignShoot.autoIndexer = AutoIndexer(self.discs)
        AutoAlignShoot.autoIndexer.execute()

        AutoAlignShoot.stop()

    @staticmethod
    def stop():
        if AutoAlignShoot.isRunning:
            AutoAlignShoot.isRunning = False
            super().stop()
            wait(50, MSEC)
            if AutoAlignShoot.autoFlywheel != None:
                AutoAlignShoot.autoFlywheel.stop()
            if AutoAlignShoot.autoIndexer != None:
                AutoAlignShoot.autoIndexer.stop()

    def alignMaintainPos(self):
        print("ATTEMPTING ALIGNMENT ...\n")
        super().execute()
        # print("ALIGNMENT COMPLETED\nCOMMENCING LAUNCHES\n")
        # self.maintainPos = True
        # self.wait = False
        # super().execute()


# ---------------------------AUTONOMOUS ROUTINES----------------------------


class RunCommands:
    isRunning = False

    def __init__(self, *commandList):
        RunCommands.commandList = commandList
        RunCommands.thread = Thread(RunCommands.run)
    
    @staticmethod
    def run():
        RunCommands.isRunning = True

        for command in RunCommands.commandList:
            command.execute()

        RunCommands.isRunning = False

    @staticmethod
    def stopAll():
        if RunCommands.isRunning == True:
            RunCommands.isRunning = False
            RunCommands.thread.stop()
            AutoDrive.stop()
            AutoAlignShoot.stop()
            AutoFlywheel.stop()
            AutoIndexer.stop()
            AutoIntake.stop()
            AutoRoller.stop()

    @staticmethod
    def sleep_for(time, unit=MSEC):
        RunCommands.thread.sleep_for(time, unit)


class TestMode:

    def __init__(self):
        print(self.__class__.__name__, "Running")
        
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
        print(self.__class__.__name__, "Running")
        
        Robot.odometry.setPose(Constants.TILE___1, Constants.TILE___0,
                               math.pi / 2)
        commandRun = RunCommands(
            AutoDrive(Constants.TILE___1, Constants.TILE_L_R, math.pi / 2, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE___1, Constants.TILE_L_S, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),

            # intake on
            AutoDrive(Constants.TILE___3, Constants.TILE___2, (5 * math.pi) / 4, 70,
                      100, True),
            AutoAlignShoot(Constants.TILE___3, Constants.TILE___2, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),
            AutoDrive(Constants.TILE_4_5, Constants.TILE_3_5, (5 * math.pi) / 4, 70,
                      100, True),
            AutoDrive(Constants.TILE___5, Constants.TILE___4, math.pi, 100,
                      100, True),

            # intake off
            AutoDrive(Constants.TILE_R_R, Constants.TILE___4, math.pi, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE_R_S, Constants.TILE___4, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),
        )


class RightAuto1:

    def __init__(self):
        print(self.__class__.__name__, "Running")
        
        Robot.odometry.setPose(Constants.TILE___5, Constants.TILE___3, math.pi)
        commandRun = RunCommands(
            AutoDrive(Constants.TILE___5, Constants.TILE___4, math.pi, 100,
                      100, True),
            AutoDrive(Constants.TILE_R_R, Constants.TILE___4, math.pi, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE_R_S, Constants.TILE___4, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),

            # intake on
            AutoDrive(Constants.TILE___3, Constants.TILE___2,
                      math.pi / 4, 70, 100),
            AutoAlignShoot(Constants.TILE___3, Constants.TILE___2, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),
            AutoDrive(Constants.TILE_1_5, Constants.TILE_0_5,
                      math.pi / 4, 70, 100),
            AutoDrive(Constants.TILE___1, Constants.TILE___0, math.pi / 2, 100, 100, True),

            # intake off
            AutoDrive(Constants.TILE___1, Constants.TILE_L_R, math.pi / 2, 100,
                      100, True),
            # AutoRoller(90),
            AutoAlignShoot(Constants.TILE___1, Constants.TILE_L_S, 0, Constants.TILE___6, 100, 100, True, timeOut=3_000),
        )


# -------------------------------UTILITIES-------------------------------


# class Graph:
#     """
#     Converted to Python from C++:
#     https://www.vexforum.com/t/graphing-on-the-v5-screen/78221
#     """

#     NUM_POINTS = 480 # pixel width of brain screen

#     def __init__(self, lines, originX, originY):
#         self.points = list()
#         self.originX = originX
#         self.originY = originY

#         class Points:
#             def __init__(self, screen):
#                 self.points = list(range(Graph.NUM_POINTS))
#                 self.screen = screen
#                 self.color  = Color.WHITE
            
#             def draw(self):
#                 self.screen.set_pen_color(self.color)
#                 for x in range(Graph.NUM_POINTS - 1):
#                     self.screen.draw_line(x, self.points[x], x + 1, self.points[x + 1])
#                     self.screen.draw_circle(x, self.points[x], 2, self.color)

#             def addPoints(self, val):
#                 for i in range(Graph.NUM_POINTS):
#                     self.points[i] = self.points[i + 1]
                
#                 self.points[Graph.NUM_POINTS - 1] = val
            
#             def setColor(self, color):
#                 self.color = color
        
#         for i in range(lines):
#             self.points.append(Points(brain.screen))

#             Thread(self.render)
    
#     def render(self):
#         while True:
#             self.draw()

#     def drawAxis(self):
#         brain.screen.set_pen_color(Color.WHITE)
#         brain.screen.draw_line(self.originX, 0, self.originX, 240)
#         brain.screen.draw_line(0, self.originY, 480, self.originY)

#         for x in range(480):
#             brain.screen.draw_line(x, self.originY + 5, x, self.originY - 5)
#         for y in range(240):
#             brain.screen.draw_line(self.originX + 5, y, self.originX - 5, y)
    
#     def draw(self):
#         brain.screen.clear_screen()
#         self.drawAxis()
#         for id in range(len(self.points)): self.points[id].draw()
#         brain.screen.render()
    
#     def addPoint(self, id, val):
#         if id < len(self.points):
#             self.points[id].addPoint(val + self.originY)
    
#     def setColor(self, id, color):
#         if id < len(self.points):
#             self.points[id].setColor(color)


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
        
        print(self.__class__.__name__, "Initialized")

    def update(self, target, current, unit):
        error = target - current
        
        self.integral += error
        if abs(error) < 0.5 or abs(error) > 5.0: self.integral = 0.0

        derivative = error - self.previousError
        self.previousError = error

        controlledValue = (self.Kp * error) + (self.Ki * self.integral) + (
            self.Kd * derivative)

        if unit == PERCENT:
            if controlledValue > 100: controlledValue = 100
            if controlledValue < -100: controlledValue = -100
        elif unit == VOLT:
            if controlledValue > 12: controlledValue = 12
            if controlledValue < -12: controlledValue = -12
        else:
            print("WRONG UNITS")

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

        self.controllerThread = Thread(self.controllerLoop)
        
        print(self.__class__.__name__, "Initialized")

    def controllerLoop(self):
        deadZoneVal = self.axisCurve(0.1)
        while (True):
            if self.controllerEnabled:
                forward = self.axisCurve(self.controller.axis3.position())
                strafe = self.axisCurve(self.controller.axis4.position())
                turn = self.axisCurve(self.controller.axis1.position())

                if abs(forward) > deadZoneVal or abs(
                        strafe) > deadZoneVal or abs(turn) > deadZoneVal:
                    RunCommands.stopAll()
                    Robot.drivetrain.drive(
                        forward * (Robot.drivetrain.driveVel / 100),
                        strafe * (Robot.drivetrain.driveVel / 100),
                        turn * (Robot.drivetrain.turnVel / 100))
                elif RunCommands.isRunning == True:
                    pass
                else:
                    Robot.drivetrain.stop()

            wait(10, MSEC)

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

        self.controller.screen.print("FWD:", Robot.flywheel.distance,
                                     "FWV:", Robot.flywheel.flywheelVel)

    def updateRow3(self):
        # FC: False MI: False
        self.controller.screen.clear_row(3)
        
        self.controller.screen.set_cursor(3, 1)

        fineControl = Robot.drivetrain.driveVel == Constants.DRIVETRAIN_FINE_CONTROL_VEL

        self.controller.screen.print("FC:", fineControl,
                                     "MI:", self.manualIndexer)

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

    def axisCurve(self, x):
        return (pow(x, 3)) / 10_000
        # if x > 0: return (x ** 2) / 100
        # return (x ** 2) / -100

    # ---------------------------BUTTON FUNCTIONS---------------------------
    """
    #### Available button commands in use
    
    self.changeDriveTrainVel()
    Robot.odometry.reset()
    self.toggleAuto()
    self.toggleDriveTrainMode()
    AutoDrive().driveToOrigin()
    """

    #  /L2 | R2\
    # / L1 | R1 \
    # |---------|
    # \_________/

    def L1_Pressed(self):
        Robot.flywheel.increaseVelocity()

    def L2_Pressed(self):
        Robot.flywheel.decreaseVelocity()

    def R1_Pressed(self):
        Robot.flywheel.toggleMotor()

    def R2_Pressed(self):
        if self.toggleManualIndexer() == False:
            if self.manualIndexer:
                Robot.indexer.push()
            else:
                # Robot.indexer.autoPush()
                pass

    """
      X
    Y   A
      B
    """

    def X_Pressed(self):
        self.printFWVelDict()

    def A_Pressed(self):
        Robot.flywheel.increaseDistance()

    def B_Pressed(self):
        Robot.flywheel.decreaseDistance()

    def Y_Pressed(self):
        AutoDrive().driveToOrigin()

    """
      ↑
    ←   →
      ↓
    """

    def Up_Pressed(self):
        self.toggleAuto()

    def Right_Pressed(self):
        self.changeDriveTrainVel()

    def Down_Pressed(self):
        Robot.odometry.reset()

    def Left_Pressed(self):
        self.toggleDriveTrainMode()

    # ----------------------BUTTON HELPER METHODS------------------------

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

    def toggleAuto(self):
        if RunCommands.isRunning == False:
            LeftAuto1()
        else:
            RunCommands.stopAll()

    def toggleDriveTrainMode(self):
        if Robot.drivetrain.getMotorMode() == BRAKE:
            Robot.drivetrain.set_stopping(COAST)
        elif Robot.drivetrain.getMotorMode() == COAST:
            Robot.drivetrain.set_stopping(BRAKE)

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

        self.thread = Thread(self.updatePose)
        
        print(self.__class__.__name__, "Initialized")

    def updatePose(self):
        print(self.__class__.__name__, "Running")
        
        inchsPerTick = Constants.INCHES_PER_TICK
        LR_Distance = Constants.LEFT_RIGHT_ODOMETRY_DISTANCE
        B_Distance = Constants.AUX_ODOMETRY_DISTANCE

        currRightVal = 0  # current encoder value for right wheel
        currLeftVal = 0  # current encoder value for left wheel
        currAuxVal = 0  # current encoder value for back wheel

        prevRightVal = 0  # previous encoder value for right wheel
        prevLeftVal = 0  # previous encoder value for left wheel
        prevAuxVal = 0  # previous encoder value for back whee

        screenStartTime = brain.timer.time(MSEC)
        screenUpdateInterval = 100

        while True:
            # anytime that x or y robot values are greater than 1,000 inches, reset encoders & pose
            if abs(self.x) > 1_000 or abs(self.y) > 1_000:
                self.reset()

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

            dtheta = inchsPerTick * ((dn2 - dn1) / LR_Distance)
            dx = inchsPerTick * ((dn1 + dn2) / 2.0)
            dy = inchsPerTick * (dn3 - ((dn2 - dn1) *
                                        (B_Distance / LR_Distance)))

            theta = self.Θ + (dtheta / 2.0)
            self.x += -dx * math.cos(-theta) + dy * math.sin(-theta)
            self.y -= -dx * math.sin(-theta) - dy * math.cos(-theta)
            self.Θ += dtheta

            screenEndTime = brain.timer.time(MSEC)

            if screenEndTime > screenStartTime + screenUpdateInterval:
                screenStartTime = screenEndTime
                myController.updateRow1()

            while brain.timer.time(MSEC) - start < 7.5:
                continue

    def stop(self):
        self.thread.stop()
        
        print(self.__class__.__name__, "Stopped")

    def getPose(self):
        return self.x, self.y, self.Θ

    def reset(self):
        self.setPose(Constants.TILE___1, 0, math.pi / 2)

    def setPose(self, newX, newY, newΘ):
        self.thread.sleep_for(100, MSEC)
        wait(50, MSEC)
        self.x = newX
        self.y = newY
        self.Θ = newΘ
        self.resetEncoders()

    def resetEncoders(self):
        self.rightEncoder.set_position(0, DEGREES)
        self.leftEncoder.set_position(0, DEGREES)
        self.auxEncoder.set_position(0, DEGREES)


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
        
        print(self.__class__.__name__, "Initialized")

    def drive(self, forward, strafe, turn):
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

    def stop(self):
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackRight.stop()
        self.motorBackLeft.stop()


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

        self.flywheelPID = PID(Constants.FLYWHEEL_KP, Constants.FLYWHEEL_KI, Constants.FLYWHEEL_KD)
        self.endgameLaunched = False
        self.flywheelVel = 1_400
        self.motorVel = self.calcMotorVel(self.flywheelVel)
        
        self.isRunning = False
        self.thread = Thread(self.run)
        self.thread.stop()

        self.distance = Constants.TILE___1

        # for 84 : 12 max: 4_200 RPM (Our robot's max)
        # for 84 : 36 max: 1_400 RPM

        self.velocityDict = {
            # need empirical data & verification
            Constants.MID_SHOT: 4_200 / 2.0 + 4_200 / 8.0,          # 2,625
            Constants.SIDE_SHOT: 4_200 * (2.0 / 3.0) + 4_200 / 8.0, # 3,325
            Constants.TILE___1: 4_200 / 4.0,                        # 1,050
            Constants.TILE___2: 4_200 / 3.0,                        # 1,400
            Constants.TILE___3: 4_200 / 2.0,                        # 2,100
            Constants.TILE___4: 4_200 * (2.0 / 3.0),                # 2,800
            Constants.TILE___5: 4_200 * (3.0 / 4.0),                # 3,150
            Constants.TILE___6: 4_200 * (4.0 / 4.0)                 # 4,200
        }
        
        print(self.__class__.__name__, "Initialized")

    def calcMotorVel(self, flywheelVel):
        return flywheelVel / Constants.FLYWHEEL_GEAR_RATIO  # 1,400 / 7 = 200 RPM

    def toggleMotor(self):
        if not self.isRunning:
            self.isRunning = True
            self.thread = Thread(self.run)
            print(self.__class__.__name__, "Running")
        else:
            self.stop()

    def run(self):
        while True:
            print("Flywheel Thread Running")
            
            controlledValue = self.flywheelPID.update(self.motorVel, self.motorGroup.velocity(RPM), VOLT)
            self.motorGroup.spin(FORWARD, controlledValue, VOLT)
            wait(10, MSEC)

    def stop(self):
        if self.isRunning:
            self.isRunning = False
            self.thread.stop()
            wait(50, MSEC)
            self.motorGroup.stop()
        else:
            print("Flywheel already stopped")
        
        print(self.__class__.__name__, "Stopped")

    def isAtSetVel(self):
        currMotorVel = self.motorGroup.velocity(RPM)

        if self.motorVel - 5 <= currMotorVel or currMotorVel <= self.motorVel + 5:
            return True
        return False

    def increaseDistance(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.TILE___1
            self.updateVel()
        else:
            self.distance += Constants.TILESIZE

            if self.distance > Constants.TILE___6:
                self.distannce = Constants.TILE___6

            self.updateVel()

    def decreaseDistance(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.TILE___1
            self.updateVel()
        else:
            self.distance -= Constants.TILESIZE

            if self.distance < Constants.TILE___1:
                self.distannce = Constants.TILE___1

            self.updateVel()

    def increaseVelocity(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.TILE___1
            self.updateVel()
        else:
            self.velocityDict[self.distance] += 50

            if self.velocityDict[self.distance] > 4_200:
                self.velocityDict[self.distance] = 4_200.0

            self.updateVel()

    def decreaseVelocity(self):
        if self.distance == Constants.MID_SHOT or self.distance == Constants.SIDE_SHOT:
            self.distance = Constants.TILE___1
            self.updateVel()
        else:
            self.velocityDict[self.distance] -= 50

            if self.velocityDict[self.distance] < 0:
                self.velocityDict[self.distance] = 0.0

            self.updateVel()

    def setDistance(self, distance):
        self.distance = distance
        self.updateVel()

    def updateVel(self):
        if self.distance in self.velocityDict.keys():
            myController.updateRow2()
            self.setVelocity(self.velocityDict[self.distance])

    def setVelocity(self, flywheelVel):
        self.flywheelVel = flywheelVel
        self.motorVel = self.calcMotorVel(self.flywheelVel)

    def launchEndgame(self):
        # TODO: add code to reverse flywheel to specific angle to launch endgame
        # TODO: add time keeping code to prevent early launch
        pass


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
        self.motor.set_velocity(Constants.TILE___5, RPM)
        self.isRunning = False

        # calculate the degrees the motor turns to move 1 chain forward
        self.degreesPerTeeth = 360 / Constants.INDEXER_GEAR_TEETH

        # calculate the total degrees the motor turns to complete 1 cycle
        self.degreesPerCycle = Constants.INDEXER_CHAIN_LINKS * self.degreesPerTeeth
        
        print(self.__class__.__name__, "Initialized")

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
        if self.motor.is_spinning() == False:
            self.motor.spin_for(FORWARD, self.degreesPerCycle, DEGREES, wait=True)

    def stop(self):
        self.motor.stop()
        
        print(self.__class__.__name__, "Stopped")


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
        # TODO: add initialization code
        
        print(self.__class__.__name__, "Initialized")

    # TODO: add any other helper methods

    def toggleMotor(self, direction=FORWARD):
        self.motor.spin(FORWARD)
        # TODO: add code to run/stop motor
        pass

    def stop(self):
        self.motor.stop()
        
        print(self.__class__.__name__, "Stopped")

    def reverseMotor(self):
        self.motor.spin(REVERSE)
        # TODO: add code to reverse motor in the event of jam
        pass


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
        
        print(self.__class__.__name__, "Initialized")

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        pass

    def flip(self, direction=FORWARD, degreesToTurn=90, wait=False):
        self.motor.spin_for(direction, degreesToTurn, DEGREES, 50, PERCENT,
                            wait)
    
    def stop(self):
        self.motor.stop()
        
        print(self.__class__.__name__, "Stopped")


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
    Robot.drivetrain.set_stopping(COAST)


# AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS


def vexcode_auton_function():
    auton_task_0 = Thread(Autonomous_Control)
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)
    Robot.drivetrain.stop()
    auton_task_0.stop()


def Autonomous_Control():
    brain.screen.print("Starting auto")
    LeftAuto1()


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
