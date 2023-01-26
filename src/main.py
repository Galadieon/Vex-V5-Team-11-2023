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

# BAD PORT: 5, 6, 7, 8, 11, 12

# Motor variables
FL = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
FR = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
BR = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
BL = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)

F1 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False) # motor closest to flywheel
# F2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
# FM = MotorGroup(flywheelLeft, flywheelRight)

# Encoder variables

rightEncoder = Encoder(brain.three_wire_port.a)
leftEncoder = Encoder(brain.three_wire_port.c)
auxEncoder = Encoder(brain.three_wire_port.e)

# Controller variables

controllerEnabled = True
controller = Controller(PRIMARY)

deadZoneVal = 0

# Odometry variables

L = 13.25
B = 5.0
D = 2.75
N = 360
circumference = math.pi * D
inchsPerTick = circumference / N

# Autonomous paths

#           x,    y,              Θ,    vel(%)
path1 = [ [ 0,    0,      math.pi/2,      25 ],
          [ 0,    5,      math.pi/2,      25 ],
          [ 0,    0,      math.pi/2,      25 ] ]

# wait for rotation sensor to fully initialize
wait(30, MSEC)


# ---------------------------CONTROLLER LOOP---------------------------

# Controller loop to handle controller readings
def controllerLoop():
    global controllerEnabled, controller, FL, FR, BL, BR, deadZoneVal
    deadZoneVal = axisCurve(3)

    printThread = Thread(printToController)

    while(True):
        if controllerEnabled:
            forward = axisCurve(controller.axis3.position())
            strafe  = axisCurve(controller.axis4.position())
            rotate  = axisCurve(controller.axis1.position())

            if abs(forward) > deadZoneVal or abs(strafe) > deadZoneVal or abs(rotate) > deadZoneVal:
                drivetrain.drive(forward, strafe, rotate)
            else:
                drivetrain.stop()
        
        wait(10, MSEC)

def printToController():
    global drivetrain
    while(True):
        # controller.screen.print("Right Encoder: ", currRightVal)
        # controller.screen.next_row()
        # controller.screen.print("Left Encoder: ", currLeftVal)
        # controller.screen.next_row()
        # controller.screen.print("Aux Encoder: ", currAuxVal)
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
    return (x ** 3) / 10000
    # if x > 0: return (x ** 2) / 100
    # return (x ** 2) / -100


# DEFAULT FUNCTIONS ---------- DEFAULT FUNCTIONS --------- DEFAULT FUNCTIONS

def when_started1():
    Driver_Control()

def Default_Motor_Speed():
    drivetrain.set_drive_velocity(motorVel, VelocityUnits.PERCENT)
    drivetrain.set_turn_velocity(motorVel, VelocityUnits.PERCENT)
    drivetrain.set_stopping(COAST)
    pass


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
    drivetrain.startAuto(path1)


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


# ---------------------------BUTTON FUNCTIONS---------------------------

def L1_Pressed():
    pass

def L2_Pressed():
    pass

def R1_Pressed():
    pass

def R2_Pressed():
    pass


def A_Pressed():
    pass

def B_Pressed():
    drivetrain.startAuto(path1)
    pass

def X_Pressed():
    pass

def Y_Pressed():
    pass


def Up_Pressed():
    # goBackToOG()
    pass

def Down_Pressed():
    pass

def Left_Pressed():
    pass

def Right_Pressed():
    pass


# ---------------------------HELPER FUNCTIONS---------------------------

def throwTheThings():
    F1.set_velocity(100, PERCENT)
    F1.spin(REVERSE)

    wait(1, SECONDS)
    F1.stop()

def tanh(x, max): # x is in inches
    n = 1.732
    return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x))) * max

def tanhTurning(x, max): # x is in radians
    n = 23.2742 # seems a little too large for turn motor value, try 3
    return ((n ** x) - (n ** (-x))) / ((n ** x) + (n ** (-x))) * max

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


# ---------------------------MECANUM DRIVETRAIN---------------------------

class MecDriveTrain:
    def __init__(self, FL, FR, BR, BL, wheelTravel, trackWidth, wheelBase, unit, gearRatio):
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
        self.Θ = math.pi /  2
        self.motorMode = BRAKE

        self.currRightVal = 0 # current encoder value for right wheel
        self.currLeftVal = 0 # current encoder value for left wheel
        self.currAuxVal = 0 # current encoder value for back wheel

        self.prevRightVal = 0 # previous encoder value for right wheel
        self.prevLeftVal = 0 # previous encoder value for left wheel
        self.prevAuxVal = 0 # previous encoder value for back wheel

    # ---------------------------AUTO AND ODOMETRY---------------------------
    
    def startAuto(self, path):
        odomThread = Thread(self.updatePosition)

        atPosition = False
        for step in path:
            while not atPosition:
                drivetrain.drive_to(step[0], step[1], step[2], step[3])
                wait(10, MSEC)

    def updatePosition(self):
        global inchsPerTick, leftEncoder, rightEncoder, auxEncoder
        while(True):
            self.prevRightVal = self.currRightVal
            self.prevLeftVal = self.currLeftVal
            self.prevAuxVal = self.currAuxVal

            self.currRightVal = -rightEncoder.value()
            self.currLeftVal = leftEncoder.value()
            self.currAuxVal = -auxEncoder.value()

            dn1 = self.currLeftVal - self.prevLeftVal
            dn2 = self.currRightVal - self.prevRightVal
            dn3 = self.currAuxVal - self.prevAuxVal

            dtheta = inchsPerTick * ((dn2 - dn1) / L)
            dx = inchsPerTick * ((dn1 + dn2) / 2.0)
            dy = inchsPerTick * (dn3 - ((dn2 - dn1) * (B / L)))

            theta = self.Θ + (dtheta / 2.0)
            self.x += -dx * math.cos(-theta) + dy * math.sin(-theta)
            self.y += -dx * math.sin(-theta) - dy * math.cos(-theta)
            self.Θ += -dtheta

            wait(10, MSEC)

    # ---------------------------DRIVE FUNCTIONS---------------------------

    def drive_to(self, xTarget, yTarget, ΘTarget, vel):
        # self.translation(xTarget, yTarget)
        # wait(10, MSEC)
        # self.rotation(ΘTarget)

        deltaX, deltaY = self.calcLocalXY(xTarget, yTarget)
        deltaTheta = ΘTarget - self.Θ

        if abs(deltaX) > 0.25 or abs(deltaY) > 0.25 or abs(deltaTheta) > 0.035:
            forward = 100 * tanh(deltaY, vel)
            strafe  = 100 * tanh(deltaX, vel)
            rotate  = 100 * tanhTurning(deltaTheta, vel)
            
            self.drive(forward, strafe, rotate)
        else:
            self.stop()
        
        wait(10, MSEC)
    
    def drive(self, forward, strafe, rotate):
        FL.set_velocity( forward + strafe + rotate, PERCENT)
        FR.set_velocity(-forward + strafe + rotate, PERCENT)
        BR.set_velocity(-forward - strafe + rotate, PERCENT)
        BL.set_velocity( forward - strafe + rotate, PERCENT)

        # FL.set_velocity(forward + strafe + rotate, PERCENT)
        # FR.set_velocity(forward - strafe - rotate, PERCENT)
        # BR.set_velocity(forward + strafe - rotate, PERCENT)
        # BL.set_velocity(forward - strafe + rotate, PERCENT)

        FL.spin(FORWARD)
        FR.spin(FORWARD)
        BR.spin(FORWARD)
        BL.spin(FORWARD)
    
    def stop(self):
        FL.stop()
        FR.stop()
        BR.stop()
        BL.stop()
    
    # ---------------------------HELPER FUNCTIONS---------------------------

    def set_drive_velocity(self, velocity, units=VelocityUnits.RPM):
        self.driveVel = velocity

    def set_turn_velocity(self, velocity, units=VelocityUnits.RPM):
        self.turnVel = velocity
    
    def set_stopping(self, mode=BrakeType.COAST):
        self.motorMode = mode
    
    def calcLocalXY(self, xTarget, yTarget):
        distance = math.sqrt(math.pow(xTarget - self.x, 2) + math.pow(yTarget - self.y, 2))
        Θd = self.calcRelAngle(xTarget, yTarget)
        φ = Θd - self.Θ + (math.pi / 2)
        localDeltaX = distance * math.cos(φ)
        localDeltaY = distance * math.sin(φ)

        return localDeltaX, localDeltaY

    def calcRelAngle(self, xTarget, yTarget):
        deltaGX = xTarget - self.x
        deltaGY = yTarget - self.y

        if deltaGX == 0 and deltaGY > 0: return math.pi / 2         # 90  deg
        if deltaGX == 0 and deltaGY < 0: return 3 * (math.pi / 2)   # 270 deg 

        ΘGTarget = math.atan(abs(deltaGY / deltaGX))

        if deltaGX > 0 and deltaGY >= 0: return ΘGTarget            # Quadrant 1 / 0 deg
        if deltaGX < 0 and deltaGY >= 0: return math.pi - ΘGTarget  # Quadrant 2 / 180 deg
        if deltaGX < 0 and deltaGY <  0: return math.pi + ΘGTarget  # Quadrant 3
        if deltaGX > 0 and deltaGY <  0: return 360 - ΘGTarget      # Quadrant 4

        return 1029300000000000084756 # return big number if an edge case is not accounted for


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

drivetrain = MecDriveTrain(FL, FR, BR, BL, 4 * math.pi, 14.097242, 11.5, INCHES, 36.0 / 84.0)

controllerThread = Thread(controllerLoop)

competition = Competition( vexcode_driver_function, vexcode_auton_function )
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

when_started1()
