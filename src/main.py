# left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
# left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
# left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
# right_motor_a = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
# right_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
# right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
# drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 381, 254, MM, 0.42857142857142855)
# encoder_a = Encoder(brain.three_wire_port.a)
# encoder_c = Encoder(brain.three_wire_port.c)
# encoder_e = Encoder(brain.three_wire_port.e)
# motor_5 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)
# distance_6 = Distance(Ports.PORT6)
# motor_group_7_motor_a = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
# motor_group_7_motor_b = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
# motor_group_7 = MotorGroup(motor_group_7_motor_a, motor_group_7_motor_b)
# controller_1 = Controller(PRIMARY)
# range_finder_g = Sonar(brain.three_wire_port.g)

# ------------------------------------------
# 
# 	Project:      Mecanum Robot Program
#	Author:       Lam Ninh
#	Created:      11/1/2022
#	Description:  VEXcode V5 Python Mecanum
#                 Project
# 
# ------------------------------------------

# Library imports
from vex import *
import math

# Begin project code
brain=Brain()

# Robot variables and configuration code

# Default values

motorVel = 75

# Motor variables
FL = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
FR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
BR = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
BL = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)

F1 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False) # motor closest to flywheel
# F2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
# FM = MotorGroup(flywheelLeft, flywheelRight)

# Encoder variables

leftEncoder = Encoder(brain.three_wire_port.a)
rightEncoder = Encoder(brain.three_wire_port.c)
auxEncoder = Encoder(brain.three_wire_port.e)

# Controller variables

controllerEnabled = True
controller = Controller(PRIMARY)

deadZoneVal = 0

# Odometry variables

L = 15.0
B = 6.0
D = 2.75
N = 90
circumference = math.pi * D
inchsPerTick = circumference / N

predictedX = 0.0
predictedY = 0.0
predictedΘ = 0.0

currRightPos = 0 # current encoder value for right wheel
currLeftPos = 0 # current encoder value for left wheel
currAuxPos = 0 # current encoder value for back wheel

prevRightPos = 0 # previous encoder value for right wheel
prevLeftPos = 0 # previous encoder value for left wheel
prevAuxPos = 0 # previous encoder value for back wheel

# wait for rotation sensor to fully initialize
wait(30, MSEC)


# ---------------------------END OF VARIABLES---------------------------

# Controller loop to handle controller readings
def controllerLoop():
    global controllerEnabled, FL, FR, BL, BR, deadZoneVal
    deadZoneVal = axisCurve(5)

    while(True):
        if controllerEnabled:
            forward = axisCurve(controller.axis3.position())
            strafe  = axisCurve(controller.axis4.position())
            rotate  = axisCurve(controller.axis1.position())

            if abs(forward) > deadZoneVal or abs(strafe) > deadZoneVal or abs(rotate) > deadZoneVal:
                FL.set_velocity(forward + strafe + rotate, PERCENT)
                FR.set_velocity(forward - strafe - rotate, PERCENT)
                BR.set_velocity(forward + strafe - rotate, PERCENT)
                BL.set_velocity(forward - strafe + rotate, PERCENT)

                FL.spin(FORWARD)
                FR.spin(FORWARD)
                BR.spin(FORWARD)
                BL.spin(FORWARD)
            else:
                FL.stop()
                FR.stop()
                BR.stop()
                BL.stop()

def axisCurve(x):
    if x > 0: return (x ** 2) / 100
    return (x ** 2) / -100


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS

def when_started1():
    Driver_Control()


# AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS ------ AUTONOMOUS FUNCTIONS

def vexcode_auton_function():
    auton_task_0 = Thread( Autonomous_Control )
    while( competition.is_autonomous() and competition.is_enabled() ):
        wait( 10, MSEC )
    FL.stop()
    FR.stop()
    BR.stop()
    BL.stop()
    F1.stop()
    auton_task_0.stop()

def Autonomous_Control():
    Default_Motor_Speed()


# DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS ------------ DRIVER FUNCTIONS

def vexcode_driver_function():
    driver_control_task_0 = Thread( Driver_Control )
    while( competition.is_driver_control() and competition.is_enabled() ):
        wait( 10, MSEC )
    FL.stop()
    FR.stop()
    BR.stop()
    BL.stop()
    F1.stop()
    driver_control_task_0.stop()

def Driver_Control():
    Default_Motor_Speed()

# ------------------------------------------------------------------------

def Default_Motor_Speed():
    drivetrain.set_drive_velocity(motorVel, VelocityUnits.PERCENT)
    drivetrain.set_turn_velocity(motorVel, VelocityUnits.PERCENT)
    drivetrain.set_stopping(COAST)
    pass






def throwTheThings():
    F1.set_velocity(100, PERCENT)
    F1.spin(REVERSE)

    wait(1, SECONDS)
    F1.stop()








def tanh(x): # x is in inches
    n = 1.732
    return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x)))

def tanhTurning(x):
    n = 23.2742
    return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x)))





def L1_Pressed():
    pass

def L2_Pressed():
    pass

def R1_Pressed():
    pass

def R2_Pressed():
    pass






def PID(encoder, target, integral, previousError, minimumVoltage, Kp, Ki, Kd):
    current = encoder.velocity(RPM)
    
    error = target - current
    
    integral += error
    if error == 0: integral = 0
    if abs(error) > 10: integral = 0
    
    derivative = error - previousError
    previousError = error
    
    motorVoltage = minimumVoltage + abs((Kp * error) + (Ki * integral) + (Kd * derivative))
    
    if motorVoltage > 12: motorVoltage = 12
    
    return integral, previousError, motorVoltage

class MecDriveTrain:
    def __init__(self, FL, FR, BR, BL, wheelTravel, trackWidth, wheelBase, unit, gearRatio):
        global currRightPos, currLeftPos, currAuxPos
        self.FL = FL
        self.FR = FR
        self.BR = BR
        self.BL = BL
        self.wheelTravel = wheelTravel
        self.trackWidth = trackWidth
        self.wheelBase = wheelBase
        self.unit = unit
        self.gearRatio = gearRatio
        self.driveVel = 25
        self.turnVel = 25
        self.x = 0
        self.y = 0
        self.theta = math.pi /  2
        self.motorMode = BRAKE

        currRightPos = 0 # current encoder value for right wheel
        currLeftPos = 0 # current encoder value for left wheel
        currAuxPos = 0 # current encoder value for back wheel
    
    def startAuto(self, path):
        atPosition = False
        for step in path:
            while not atPosition:
                drivetrain.set_drive_velocity(path[step][3])
                drivetrain.set_turn_velocity(path[step][4])
                drivetrain.drive_to(path[step][0], step][1], step][2])
                wait(10, MSEC)

    def updatePostion(self):
        global leftEncoder, rightEncoder, auxEncoder, currRightPos, currLeftPos, currAuxPos, prevRightPos, prevLeftPos, prevAuxPos, predictedX, predictedY, predictedΘ
        prevRightPos = currRightPos
        prevLeftPos = currLeftPos
        prevAuxPos = currAuxPos

        currRightPos = rightEncoder.value()
        currLeftPos = leftEncoder.value()
        currAuxPos = auxEncoder.value()

        dn1 = currLeftPos - prevLeftPos
        dn2 = currRightPos - prevRightPos
        dn3 = currAuxPos - prevAuxPos

        dtheta = inchsPerTick * ((dn2 - dn1) / L)
        dx = inchsPerTick * ((dn1 + dn2) / 2.0)
        dy = inchsPerTick * (dn3 - ((dn2 - dn1) * (B / L)))

        theta = predictedΘ + (dtheta / 2.0)
        predictedX += -dx * math.cos(-theta) + dy * math.sin(-theta)
        predictedY += -dx * math.sin(-theta) - dy * math.cos(-theta)
        predictedΘ += dtheta

    def drive_to(self, xTarget, yTarget, thetaTarget):
        while True:
            self.updatePostion()
            self.translation()
            self.updatePostion()
            self.rotation()

            deltaX = xTarget - self.x
            deltaY = yTarget - self.y
            deltaTheta = thetaTarget - self.theta

            if abs(deltaX) > 0.25 or abs(deltaY) > 0.25 or abs(deltaTheta) > 0.035:
                strafe = 100 * tanh(deltaX)
                forward = 100 * tanh(deltaY)
                rotate = 100 * tanhTurning(deltaTheta)

                FL.set_velocity(forward + strafe + rotate, PERCENT)
                FR.set_velocity(forward - strafe - rotate, PERCENT)
                BR.set_velocity(forward + strafe - rotate, PERCENT)
                BL.set_velocity(forward - strafe + rotate, PERCENT)

                FL.spin(FORWARD)
                FR.spin(FORWARD)
                BR.spin(FORWARD)
                BL.spin(FORWARD)
            else:
                FL.stop()
                FR.stop()
                BR.stop()
                BL.stop()
    
    def translation(self, x, y):
        pass
    
    def rotation(self, Θ):
        pass

    def set_drive_velocity(self, velocity, units=VelocityUnits.RPM):
        self.driveVel = velocity

    def set_turn_velocity(self, velocity, units=VelocityUnits.RPM):
        self.turnVel = velocity
    
    def set_stopping(self, mode=BrakeType.COAST):
        self.motorMode = mode


# wait for rotation sensor to fully initialize
wait(30, MSEC)

# END OF METHODS

# system event handlers
controller.buttonL1.pressed(L1_Pressed)
controller.buttonL2.pressed(L2_Pressed)
controller.buttonR1.pressed(R1_Pressed)
controller.buttonR2.pressed(R2_Pressed)

drivetrain = MecDriveTrain(FL, FR, BR, BL, 4 * math.pi, 14.097242, 11.5, INCHES, 36.0 / 84.0)

controllerThread = Thread(controllerLoop)

competition = Competition( vexcode_driver_function, vexcode_auton_function )
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

when_started1()