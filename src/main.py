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

# ---------------------------CLASS DECLARATION---------------------------

# ------------------------------AUTONOMOUS-------------------------------


class RunCommands:
    stopAuto = False
    pauseAuto = False
    autoIsRunning = False

    def __init__(self, *commandList):
        RunCommands.autoIsRunning = True

        for command in commandList:
            while RunCommands.pauseAuto:
                pass
            if RunCommands.stopAuto:
                RunCommands.stopAuto = False
                RunCommands.autoIsRunning = False
                return
            command.execute()

        RunCommands.autoIsRunning = False

    @staticmethod
    def stop():
        RunCommands.stopAuto = True

    @staticmethod
    def pause():
        RunCommands.pauseAuto = True

    @staticmethod
    def unpause():
        RunCommands.pauseAuto = False


class TestMode:

    def __init__(self):
        commandRun = RunCommands(AutoDrive(0, 10, math.pi / 2, 25, 25))


# -------------------------------UTILITIES-------------------------------


class Constants:
    """
    ### Constants class - class to hold final constants

    This class holds constants that won't be changed while running.

    #### Arguments:
        None

    #### Returns:
        None

    #### Examples:
        Constants.LEFT_DRIVE_TRAIN_FORWARD\\
        Constants.INDEXER_PORT\\
        Constants.WHEEL_TRAVEL
    """

    LEFT_DRIVE_TRAIN_FORWARD = Ports.PORT1
    RIGHT_DRIVE_TRAIN_FORWARD = Ports.PORT2
    RIGHT_DRIVE_TRAIN_BACK = Ports.PORT10
    LEFT_DRIVE_TRAIN_BACK = Ports.PORT9

    # subject to change
    INDEXER_PORT = Ports.PORT11
    FLYWHEEL_PORT1 = Ports.PORT15
    FLYWHEEL_PORT2 = Ports.PORT16
    INTAKE_PORT = Ports.PORT20

    RIGHT_ENCODER = Encoder(brain.three_wire_port.e)
    LEFT_ENCODER = Encoder(brain.three_wire_port.a)
    AUX_ENCODER = Encoder(brain.three_wire_port.c)

    WHEEL_TRAVEL = 4 * math.pi
    TRACK_WIDTH = 14.097242
    WHEEL_BASE = 11.5

    LEFT_RIGHT_ODOMETRY_DISTANCE = 13.5
    AUX_ODOMETRY_DISTANCE = 5.0
    ODOMETRY_DIAMETER = 2.75
    QUADRATURE_ENCODER_TICKS = 360
    ODOMETRY_CIRCUMFERENCE = math.pi * ODOMETRY_DIAMETER
    INCHES_PER_TICK = ODOMETRY_CIRCUMFERENCE / QUADRATURE_ENCODER_TICKS

    FLYWHEEL_KP = 1
    FLYWHEEL_KI = 0
    FLYWHEEL_KD = 0

    DRIVETRAIN_FORWARD_KP = 1
    DRIVETRAIN_FORWARD_KI = 0
    DRIVETRAIN_FORWARD_KD = 0

    DRIVETRAIN_STRAFE_KP = 1
    DRIVETRAIN_STRAFE_KI = 0
    DRIVETRAIN_STRAFE_KD = 0

    DRIVETRAIN_TURN_KP = 1
    DRIVETRAIN_TURN_KI = 0
    DRIVETRAIN_TURN_KD = 0


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

    def __init__(self, Kp=1, Ki=0, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prevError = 0.0
        self.integral = 0.0

    def update(self, target, current):
        error = target - current

        self.integral += error
        if error == 0 or abs(error) > 2.0: self.integral = 0.0

        derivative = error - self.previousError
        self.previousError = error

        controlledValue = abs((self.Kp * error) + (self.Ki * self.integral) +
                              (self.Kd * derivative))

        if controlledValue > 100: controlledValue = 100
        if controlledValue < -100: controlledValue = -100

        return controlledValue


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
        self.x = 0
        self.y = 0
        self.Θ = math.pi / 2

        self.rightEncoder = rightEncoder
        self.leftEncoder = leftEncoder
        self.auxEncoder = auxEncoder

        self.threadIsRunning = False
        self.threadIsPaused = False

    def updatePose(self):
        inchsPerTick = Constants.INCHES_PER_TICK
        LR_Distance = Constants.LEFT_RIGHT_ODOMETRY_DISTANCE
        B_Distance = Constants.AUX_ODOMETRY_DISTANCE

        currRightVal = 0  # current encoder value for right wheel
        currLeftVal = 0  # current encoder value for left wheel
        currAuxVal = 0  # current encoder value for back wheel

        prevRightVal = 0  # previous encoder value for right wheel
        prevLeftVal = 0  # previous encoder value for left wheel
        prevAuxVal = 0  # previous encoder value for back whee

        self.threadIsRunning = True

        while (self.threadIsRunning):
            if self.threadIsPaused:
                wait(1, MSEC)
                continue

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
        wait(5, MSEC)
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


class AutoDrive:
    """
    ### AutoDrive class - creates an auto drive object

    This class is used to command drivetrain to move to global target.

    #### Arguments:
        xTarget = The global x target
        xTarget = The global y target
        ΘTarget = The global Θ target
        driveVel = The max drive velocity
        turnVel = The max turn velocity

    #### Returns:
        A new AutoDrive command object

    #### Examples:
        RunCommands(AutoDrive(0, 10, math.pi / 2, 25, 25), ...)
    """

    def __init__(self, xTarget, yTarget, ΘTarget, driveVel, turnVel):
        self.xTarget = xTarget
        self.yTarget = yTarget
        self.ΘTarget = ΘTarget
        self.driveVel = driveVel
        self.turnVel = turnVel

    def calcLocalXY(self):
        robotX, robotY, robotΘ = Robot.odometry.getPose()

        deltaX = self.xTarget - robotX
        deltaY = self.yTarget - robotY

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

    @staticmethod
    def driveToOrigin():
        atTarget = False
        
        while not atTarget: 
            atTarget = AutoDrive.driveTo([0, 0], math.pi / 2, 25, 25)
            
            wait(10, MSEC)

    @staticmethod
    def driveTo(localXY, ΘTarget: float, driveVel: float,
                turnVel: float):
        '''
        ### AutoDrive
        '''

        deltaX, deltaY = localXY
        deltaTheta = ΘTarget - Robot.odometry.Θ

        if abs(deltaX) < 0.25 and abs(deltaY) < 0.25 and abs(
                deltaTheta) < 0.035:
            print("AT TARGET")
            return True

        forward = 10 if deltaY > 1 else -10 if deltaY < -1 else 0
        strafe = 10 if deltaX > 1 else -10 if deltaX < -1 else 0
        turn = 10 if deltaTheta > 0.045 else -10 if deltaTheta < -0.045 else 0

        Robot.drivetrain.autoDrive(forward, strafe, turn)

        return False

    def execute(self):
        atTarget = False
        
        while not atTarget: 
            atTarget = self.driveTo(self.calcLocalXY(), self.ΘTarget,
                                 self.driveVel, self.turnVel)
            
            wait(10, MSEC)


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

        self.registerEventHandlers()
        self.run()

    def run(self):
        Thread(self.printToController)

        Thread(self.controllerLoop)

    def controllerLoop(self):
        deadZoneVal = self.axisCurve(0.1)
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
                elif RunCommands.autoIsRunning == True:
                    pass
                else:
                    Robot.drivetrain.stop()

            wait(10, MSEC)

    def printToController(self):
        while (True):
            # self.controller.screen.print("Right Encoder: ", Robot.drivetrain.rightEncoder.value())
            # self.controller.screen.next_row()
            # self.controller.screen.print("Left Encoder: ", Robot.drivetrain.leftEncoder.value())
            # self.controller.screen.next_row()
            # self.controller.screen.print("Aux Encoder: ", Robot.drivetrain.auxEncoder.value())
            # self.controller.screen.next_row()

            robotX, robotY, robotΘ = Robot.odometry.getPose()

            self.controller.screen.print("Global X: ", robotX)
            self.controller.screen.next_row()
            self.controller.screen.print("Global Y: ", robotY)
            self.controller.screen.next_row()
            self.controller.screen.print("Global Θ: ", math.degrees(robotΘ))
            self.controller.screen.next_row()

            wait(100, MSEC)

            self.controller.screen.clear_screen()
            self.controller.screen.set_cursor(1, 1)

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

    def L1_Pressed(self):
        pass

    def L2_Pressed(self):
        if Robot.drivetrain.driveVel == 100:
            Robot.drivetrain.set_turn_velocity(50)
            Robot.drivetrain.set_drive_velocity(50)
        else:
            Robot.drivetrain.set_turn_velocity(100)
            Robot.drivetrain.set_drive_velocity(100)

    def R1_Pressed(self):
        pass

    def R2_Pressed(self):
        pass

    def A_Pressed(self):
        if RunCommands.autoIsRunning == False:
            TestMode()
        else:
            RunCommands.stop()

    def B_Pressed(self):
        if Robot.drivetrain.getMotorMode() == BRAKE:
            Robot.drivetrain.set_stopping(COAST)
        elif Robot.drivetrain.getMotorMode() == COAST:
            Robot.drivetrain.set_stopping(BRAKE)

    def X_Pressed(self):
        Robot.odometry.resetPose()

    def Y_Pressed(self):
        pass

    def Up_Pressed(self):
        AutoDrive.driveToOrigin()

    def Down_Pressed(self):
        pass

    def Left_Pressed(self):
        pass

    def Right_Pressed(self):
        pass


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
        drivetrain1 = MecanumDriveTrain(Constants.LEFT_DRIVE_TRAIN_FORWARD,\\
        Constants.RIGHT_DRIVE_TRAIN_FORWARD,\\
        Constants.RIGHT_DRIVE_TRAIN_BACK,\\
        Constants.LEFT_DRIVE_TRAIN_BACK)
    """

    def __init__(self, FL, FR, BR, BL):
        self.motorFrontLeft = Motor(FL, GearSetting.RATIO_18_1, False)
        self.motorFrontRight = Motor(FR, GearSetting.RATIO_18_1, False)
        self.motorBackRight = Motor(BR, GearSetting.RATIO_18_1, False)
        self.motorBackLeft = Motor(BL, GearSetting.RATIO_18_1, False)

        self.driveVel = 100
        self.turnVel = 100
        self.motorMode = COAST

        self.forwardPID = PID(Kp=1)
        self.strafePID = PID(Kp=1)
        self.turnPID = PID(Kp=1)

        Robot.odometry.resetEncoders()

        # Start odometry thread to run independetly of other threads
        Thread(Robot.odometry.updatePose)

    def drive(self, forward, strafe, turn):
        '''
        ### TeleOp drive
        '''

        self.motorFrontLeft.set_velocity(forward + strafe + turn, PERCENT)
        self.motorFrontRight.set_velocity(-1 * (-forward + strafe + turn),
                                          PERCENT)
        self.motorBackRight.set_velocity(-forward - strafe + turn, PERCENT)
        self.motorBackLeft.set_velocity(forward - strafe + turn, PERCENT)

        self.motorFrontLeft.spin(FORWARD)
        self.motorFrontRight.spin(FORWARD)
        self.motorBackRight.spin(FORWARD)
        self.motorBackLeft.spin(FORWARD)

    def autoDrive(self, forward, strafe, turn):
        self.motorFrontLeft.set_velocity(forward + strafe + turn, PERCENT)
        self.motorFrontRight.set_velocity(-1 * (-forward + strafe + turn),
                                          PERCENT)
        self.motorBackRight.set_velocity(-forward - strafe + turn, PERCENT)
        self.motorBackLeft.set_velocity(forward - strafe + turn, PERCENT)

        # self.motorFrontLeft.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        # self.motorFrontRight.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        # self.motorBackRight.spin_for(FORWARD, 0.01, SECONDS, wait=False)
        # self.motorBackLeft.spin_for(FORWARD, 0.01, SECONDS, wait=False)

        self.motorFrontLeft.spin(FORWARD, wait=False)
        self.motorFrontRight.spin(FORWARD, wait=False)
        self.motorBackRight.spin(FORWARD, wait=False)
        self.motorBackLeft.spin(FORWARD, wait=False)

        return False

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
        self.motorGroup = MotorGroup(*[motors])
        self.flywheelPID = PID(Kp=1)
        self.endgameLaunched = False

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        pass

    def launchEndgame(self):
        # TODO: add code to reverse flywheel to specific angle to launch endgame
        # TODO: add time keeping code to prevent early launch
        pass

    def changeSpeed(self):
        # TODO: add code to change motor speed low to high and low again
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
        self.motor = Motor(motor, GearSetting.RATIO_18_1, False)

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        pass

    def reverseMotor(self):
        # TODO: add code to reverse motor in case of jam when holding button
        pass

    def changeSpeed(self):
        # TODO: (maybe) add code to change motor speed
        pass


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

    def toggleMotor(self):
        # TODO: add code to run/stop motor
        pass

    def reverseMotor(self):
        # TODO: add code to reverse motor in the event of jam
        pass


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

    # Subsystem variable instantiation and initialization
    drivetrain = MecanumDriveTrain(Constants.LEFT_DRIVE_TRAIN_FORWARD,
                                   Constants.RIGHT_DRIVE_TRAIN_FORWARD,
                                   Constants.RIGHT_DRIVE_TRAIN_BACK,
                                   Constants.LEFT_DRIVE_TRAIN_BACK)

    indexer = Indexer(Constants.INDEXER_PORT)

    flywheel = Flywheel(Constants.FLYWHEEL_PORT1)

    intake = Intake(Constants.INTAKE_PORT)

    # Utility variable instantiation and initialization
    odometry = Odometry(Constants.RIGHT_ENCODER, Constants.LEFT_ENCODER,
                        Constants.AUX_ENCODER)


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS


def non_competition_start():
    Driver_Control()


def Default_Motor_Speed():
    Robot.drivetrain.set_drive_velocity(100, VelocityUnits.PERCENT)
    Robot.drivetrain.set_turn_velocity(100, VelocityUnits.PERCENT)
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
    TestMode()


# DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS


def vexcode_driver_function():
    driver_control_task_0 = Thread(Driver_Control)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    Robot.drivetrain.stop()
    driver_control_task_0.stop()


def Driver_Control():
    Default_Motor_Speed()


# ---------------------------REQUIRED CODE---------------------------

# wait for rotation sensor to fully initialize
wait(30, MSEC)

myController = MyController()

competition = Competition(vexcode_driver_function, vexcode_auton_function)

# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

non_competition_start()