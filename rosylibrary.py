# Think Create Learn
#
# Robot controller code
#
# Copyright (C) 2017 Llewelyn Fernandes.  This code and associated notes is for use by Think Create Learn
# students only and must not be shared or distributed in any way.  
#
# Code for simple robot programming

# ======================================================================================================
# Import the modules we will need
# ======================================================================================================

import pygame, sys
import time
import piconzero3 as pz
import math    
import RPi.GPIO as GPIO, time 



# ======================================================================================================
# Set up constants
# ======================================================================================================

# Define our motor ids for the Picon Zero controller
MOTOR_LEFT = 1
MOTOR_RIGHT = 0

# Define the pins we connected the line following sensor to 
LINE_FOLLOW_PIN_LEFT = 4
LINE_FOLLOW_PIN_CENTRE = 17
LINE_FOLLOW_PIN_RIGHT = 18

# Line following values
BLACK = 1
WHITE = 0

# Define Sonar Pin (Uses same pin for both Ping and Echo)
SONAR_PIN = 20 

# Define the values we will need for ultrasonics
SONAR_SPEED_OF_SOUND = 343                                            # metres per second 
SONAR_MAX_DISTANCE = 1                                                # metres  
SONAR_MAX_WAIT = float(SONAR_MAX_DISTANCE) / SONAR_SPEED_OF_SOUND * 2 # seconds  

# Spin direction
CLOCKWISE = 1
ANTICLOCKWISE = -1



# ======================================================================================================
# Set up global variables
# ======================================================================================================

# Flags to indicate robot state
_started = False
#_calibratedSpeed = False
#_calibratedSpins = False
_tracing = False
_forceCalibration = False

# Default settings for Edwina (a slow robot)
robotName = "Unknown"
wheelDiameterInMillimetres = 32
wheelSpacingInMillimetres = 111   # distance between wheels (centre to centre)
motorMaxSpeedRPM = 80

# Calibration settings for Edwina (a slow robot)
_calibratedSpeedMillimetresPerSecond = [1, 11, 22, 33, 44, 56, 67, 78, 89, 111]
_calibratedSpinsPerSecond = 0.333



# ======================================================================================================
# Define Private Functions
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Private function to check that the robot has been initialised
# ------------------------------------------------------------------------------------------------------
def _check():
    if not _started:
        raise RosiException("Oops!  You can't use your robot until you have started it")
    return True

# ------------------------------------------------------------------------------------------------------
# Private function to output a trace message if _tracing is on.
# ------------------------------------------------------------------------------------------------------
def _trace(*args):
    if _tracing:
        message = "..."
        for arg in args:
            message += str(arg)
        print (message)
  
# ------------------------------------------------------------------------------------------------------
# Private function to wait for a short time to buffer between actions (makes movements more accurate)
# ------------------------------------------------------------------------------------------------------
def _bufferWait():
    time.sleep(0.5)

# ------------------------------------------------------------------------------------------------------
# Private function to move a set distance.  Robot must have correct calibration settings beforehand.
# ------------------------------------------------------------------------------------------------------
def _moveDistance(motorSpeed=100, metres=0, centimetres=0, millimetres=0, indent=1):
    if _check():
        # Convert distance to seconds
        seconds = 0
        millimetresPerSecond = _calibratedSpeedMillimetresPerSecond[int((abs(motorSpeed)-1)/10)]
        distanceMillimetres = metres * 1000 + centimetres*10 + millimetres
        seconds = float(distanceMillimetres) / millimetresPerSecond
        _trace ("{:{}}Move Distance {} millimetres (calculated as {} seconds at speed {})".format(" ", indent*3, distanceMillimetres, round(seconds,3), motorSpeed))

        # Move for that many seconds
        _moveTime(motorSpeed, seconds, indent+1)

# ------------------------------------------------------------------------------------------------------
# Private function to move the robot forward or backward for a given time period
# ------------------------------------------------------------------------------------------------------
def _moveTime(motorSpeed=100, seconds=0, indent=1):
    if _check():
        _trace ("{:{}}Move for {} seconds at speed {}".format(" ", indent*3, round(seconds,3), motorSpeed))
        pz.setMotor(MOTOR_LEFT, motorSpeed)
        pz.setMotor(MOTOR_RIGHT, motorSpeed)
        time.sleep(seconds)
        stop()
        _bufferWait()

# ------------------------------------------------------------------------------------------------------
# Private function to set the motors turning at the given speed
# ------------------------------------------------------------------------------------------------------
def _turnMotors(leftMotorSpeed=100, rightMotorSpeed=100, indent=1):
    if _check():
        _trace ("{:{}}Turn motors at L={} R={}".format(" ", indent*3, leftMotorSpeed, rightMotorSpeed))
        pz.setMotor(MOTOR_LEFT, leftMotorSpeed)
        pz.setMotor(MOTOR_RIGHT, rightMotorSpeed)

# ------------------------------------------------------------------------------------------------------
# Private function to spin the robot on the spot by a given angle.  Robot must have correct calibration 
# settings beforehand. -ve is anticlockwise, +ve clockwise
# ------------------------------------------------------------------------------------------------------
def _spinAngle(degrees, indent=1):     
    if _check():
        pz.setMotor(MOTOR_LEFT, int(100 * math.copysign(1,degrees)))
        pz.setMotor(MOTOR_RIGHT, int(100 * math.copysign(1,-degrees)))
        seconds = abs(degrees) / 360.0 / _calibratedSpinsPerSecond

        _trace ("{:{}}Spin Angle {} for {} seconds".format(" ", indent*3, degrees, round(seconds,2)))
        time.sleep(seconds)                                        
        stop()
        _bufferWait()

# ------------------------------------------------------------------------------------------------------
# Private function to spin the robot on the spot for a given time
# -ve is anticlockwise, +ve clockwise
# ------------------------------------------------------------------------------------------------------
def _spinTime(seconds, direction=1, indent=1):     
    if _check():
        pz.setMotor(MOTOR_LEFT, int(100 * math.copysign(1,direction)))
        pz.setMotor(MOTOR_RIGHT, int(100 * math.copysign(1,-direction)))

        _trace ("{:{}}Spin for {} seconds".format(" ", indent*3, seconds))
        time.sleep(seconds)                                        
        stop()
        _bufferWait()
       
# ------------------------------------------------------------------------------------------------------
# Private function to read the 3 values from the line sensor
# ------------------------------------------------------------------------------------------------------
def _readLightSensor(indent=1):  
    if _check():
        # Read from the GPIO pins
        left = GPIO.input(LINE_FOLLOW_PIN_LEFT)
        centre = GPIO.input(LINE_FOLLOW_PIN_CENTRE)
        right = GPIO.input(LINE_FOLLOW_PIN_RIGHT)
        _trace ("{:{}}Seeing {} {} {}".format(" ", indent*3, _blackWhite(left), _blackWhite(centre), _blackWhite(right)))
        return (left, centre, right)

# ------------------------------------------------------------------------------------------------------
# Private function to translate the value from the sensor to a friendly name
# ------------------------------------------------------------------------------------------------------
def _blackWhite(bw):
    if bw==BLACK:
        return "BLACK"
    else:
        return "WHITE"

# ------------------------------------------------------------------------------------------------------
# Private function to read the signal bounce time from the ultrasonic sensor 
# ------------------------------------------------------------------------------------------------------
def _readSonarTime(indent=1): 
    if _check():
        # Send 10us pulse to trigger the ultrasonic module
        GPIO.setup(SONAR_PIN, GPIO.OUT)
        GPIO.output(SONAR_PIN, True)
        time.sleep(0.00001)
        GPIO.output(SONAR_PIN, False)

        # Switch to input mode
        GPIO.setup(SONAR_PIN, GPIO.IN)
        
        # Start the clock
        start = time.time()

        # Wait for sensor to go high - pulse sent
        count=time.time()
        while GPIO.input(SONAR_PIN)==0 and time.time()-count<SONAR_MAX_WAIT:
            start = time.time()

        # Wait for sensor to go low - echo has come back
        count=time.time()
        stop=count
        while GPIO.input(SONAR_PIN)==1 and time.time()-count<SONAR_MAX_WAIT:
            stop = time.time()
            
        # Calculate pulse length
        elapsed = stop-start

        _trace ("{:{}}Time {}".format(" ", indent*3, elapsed))

        return elapsed

# ------------------------------------------------------------------------------------------------------
# Private function to read the distance from the ultrasonic sensor  
# ------------------------------------------------------------------------------------------------------
def _readSonarDistance(indent=1): 
    if _check():
        t = _readSonarTime()
    
        # Distance is time x speed of sound
        distance = t * SONAR_SPEED_OF_SOUND * 100
        
        # That was the distance there and back so halve the value
        distance = distance / 2

        _trace ("{:{}}Distance {}".format(" ", indent*3, distance))
        
        return distance  

# ------------------------------------------------------------------------------------------------------
# Private function to read settings from a file
# ------------------------------------------------------------------------------------------------------
def _readSettings():
    global wheelDiameterInMillimetres
    global wheelSpacingInMillimetres
    global motorMaxSpeedRPM
    global robotName

    try:
        f = open("rosi.settings")
        lines = f.readlines()
        _trace("Reading settings from rosi.settings:")
        for line in lines:
            setting = line.split("=")
            if len(setting)==2:
                _trace(setting)
                name = setting[0].strip().lower()
                value = setting[1].strip()
                if name == "wheelDiameterInMillimetres".lower():
                    wheelDiameterInMillimetres = int(value)
                elif name == "wheelSpacingInMillimetres".lower():
                    wheelSpacingInMillimetres = int(value)
                elif name == "motorMaxSpeedRPM".lower():
                    motorMaxSpeedRPM = int(value)
                elif name == "calibrateSpeed".lower():
                    params = value.split(",")
                    calibrateSpeed(millimetres=int(params[0]), seconds=int(params[1]))
                elif name == "calibrateSpins".lower():
                    params = value.split(",")
                    calibrateSpins(spins=float(params[0]), seconds=int(params[1]))
        f.close()
    except FileNotFoundError:
        pass



# ======================================================================================================
# Exception Handling
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Exception to raise if something goes wrong
# ------------------------------------------------------------------------------------------------------
class RosiException(Exception):
    def __init__(self, value):
        self.value = value
    
    def __str__(self):
        return repr(self.value)



# ======================================================================================================
# Define Public Functions: Setting up and Resetting
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Start the robot
# ------------------------------------------------------------------------------------------------------
def start():
    #global _calibratedSpeed
    #global _calibratedSpins
    global _started

    # Attempt to read settings from file (if no file then we stick with the defaults)
    _readSettings()

    # Set up the hardware
    reset()
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)            

    # Set up the GPIO pins for the line follower
    chan_list = [LINE_FOLLOW_PIN_LEFT,LINE_FOLLOW_PIN_CENTRE, LINE_FOLLOW_PIN_RIGHT]  
    GPIO.setup(chan_list, GPIO.IN)    

    # Set the flag to say that the robot started successfully
    _started = True

# ------------------------------------------------------------------------------------------------------
# Stop the robot and turn off the controller
# ------------------------------------------------------------------------------------------------------
def finish():
    pz.stop()

# ------------------------------------------------------------------------------------------------------
# Reset the robot, e.g. if something went wrong
# ------------------------------------------------------------------------------------------------------
def reset():
    # Reset the Picon Zero to its defaults
    pz.stop()
    pz.init()



# ======================================================================================================
# Define Public Functions: Moving
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Stop the robot from moving
# ------------------------------------------------------------------------------------------------------
def stop():
    pz.setMotor(MOTOR_LEFT,0)
    pz.setMotor(MOTOR_RIGHT,0)

# ------------------------------------------------------------------------------------------------------
# Move the robot forward for a given time period
# ------------------------------------------------------------------------------------------------------
def forwardTime(motorSpeed=100, seconds=0):
    _trace ("Command: Forward {} seconds".format(seconds))
    _moveTime(motorSpeed, seconds)

# ------------------------------------------------------------------------------------------------------
# Move the robot backward for a given time period
# ------------------------------------------------------------------------------------------------------
def backwardTime(motorSpeed=100, seconds=0):
    _trace ("Command: Backward {} seconds".format(seconds))
    _moveTime(-motorSpeed, seconds)

# ------------------------------------------------------------------------------------------------------
# Move the robot forward a given distance
# ------------------------------------------------------------------------------------------------------
def forwardDistance(motorSpeed=100, metres=0, centimetres=0, millimetres=0):
    _trace ("Command: Forward Distance {} metres, {} centimetres, {} millimetres at speed {}".format(metres, centimetres, millimetres, motorSpeed))
    _moveDistance(motorSpeed, metres, centimetres, millimetres)

# ------------------------------------------------------------------------------------------------------
# Move the robot backward a given distance
# ------------------------------------------------------------------------------------------------------
def backwardDistance(motorSpeed=100, metres=0, centimetres=0, millimetres=0):
    _trace ("Command: Backward Distance {} metres, {} centimetres, {} millimetres at speed {}".format(metres, centimetres, millimetres, motorSpeed))
    _moveDistance(-motorSpeed, metres, centimetres, millimetres)

# ------------------------------------------------------------------------------------------------------
# Spin the robot right on the spot
# ------------------------------------------------------------------------------------------------------
def spinRight():
    _trace ("Command: Spin Right")
    _spinAngle(90)

# ------------------------------------------------------------------------------------------------------
# Spin the robot left on the spot
# ------------------------------------------------------------------------------------------------------
def spinLeft():
    _trace ("Command: Spin Left")
    _spinAngle(-90)

# ------------------------------------------------------------------------------------------------------
# Spin the robot a specific angle
# -ve is anticlockwise, +ve clockwise
# ------------------------------------------------------------------------------------------------------
def spinAngle(angle):
    _trace ("Command: Spin Angle")
    _spinAngle(angle)

# ------------------------------------------------------------------------------------------------------
# Spin the robot for the specified time
# -1 is anticlockwise, +1 clockwise
# ------------------------------------------------------------------------------------------------------
def spinTime(seconds, direction=1):
    _trace ("Command: Spin Time")
    _spinTime(seconds, direction)   

# ------------------------------------------------------------------------------------------------------
# Start the motors turning at the given speeed (don't stop them)
# ------------------------------------------------------------------------------------------------------
def turnMotors(leftMotorSpeed=100, rightMotorSpeed=100):
    _trace ("Command: turnMotors R={} L={}".format(leftMotorSpeed, rightMotorSpeed))
    _turnMotors(int(leftMotorSpeed), int(rightMotorSpeed))
    


# ======================================================================================================
# Define Public Functions: Detecting using the Light Sensor
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Check if the light sensor is seeing BLACK,BLACK,BLACK
# ------------------------------------------------------------------------------------------------------
def lightSensorSeeingBlob():
    _trace ("Command: Is Light Sensor Seeing Blob?")    
    return _readLightSensor()==(1,1,1)

# ------------------------------------------------------------------------------------------------------
# Check if the light sensor is seeing the specified values
# ------------------------------------------------------------------------------------------------------
def lightSensorSeeing(left, centre, right):
    _trace ("Command: Is Light Sensor Seeing {} {} {}?".format(_blackWhite(left), _blackWhite(centre), _blackWhite(right)))    
    return _readLightSensor()==(left, centre, right)

# ------------------------------------------------------------------------------------------------------
# Read the values from the light sensor and return as a tuple (left, centre, right)
# ------------------------------------------------------------------------------------------------------
def readLightSensor():
    _trace ("Command: readLightSensor")    
    return _readLightSensor()



# ======================================================================================================
# Define Public Functions: Detecting using the Ultrasonics Sensor
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Read the time for the sonar signal to bouce to and from the object
# ------------------------------------------------------------------------------------------------------
def readSonarTime(): 
    _trace ("Command: readSonarTime")    
    return _readSonarTime()

# ------------------------------------------------------------------------------------------------------
# Read distance from ultrasonic sensor  
# ------------------------------------------------------------------------------------------------------
def readSonarDistance(indent=1):
    _trace ("Command: _readSonarDistance")    
    return _readSonarDistance()



# ======================================================================================================
# Define Public Functions: Calibrating
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Calibrate the robot linear speed
# ------------------------------------------------------------------------------------------------------
def calibrateSpeed(millimetres, seconds, motorSpeed=0):
    #global _calibratedSpeed
    global _calibratedSpeedMillimetresPerSecond

    millimetresPerSecond = millimetres / seconds

    # If motorSpeed is 0 (i.e. not provided) then fill the whole list based on extrapolation from 100
    if motorSpeed == 0:
        _calibratedSpeedMillimetresPerSecond[9] = round(millimetresPerSecond)      # 91-100
        _calibratedSpeedMillimetresPerSecond[8] = round(millimetresPerSecond*8/10) # 81-90
        _calibratedSpeedMillimetresPerSecond[7] = round(millimetresPerSecond*7/10) # 71-80
        _calibratedSpeedMillimetresPerSecond[6] = round(millimetresPerSecond*6/10) # 61-70
        _calibratedSpeedMillimetresPerSecond[5] = round(millimetresPerSecond*5/10) # 51-60
        _calibratedSpeedMillimetresPerSecond[4] = round(millimetresPerSecond*4/10) # 41-50
        _calibratedSpeedMillimetresPerSecond[3] = round(millimetresPerSecond*3/10) # 31-40
        _calibratedSpeedMillimetresPerSecond[2] = round(millimetresPerSecond*2/10) # 21-30
        _calibratedSpeedMillimetresPerSecond[1] = round(millimetresPerSecond*1/10) # 11-20
        _calibratedSpeedMillimetresPerSecond[0] = 1                         # 1-10 - we know this is too slow to move!
    else:
        _calibratedSpeedMillimetresPerSecond[int((motorSpeed-1)/10)] = millimetresPerSecond
    #_calibratedSpeed = True        
    
# ------------------------------------------------------------------------------------------------------
# Calibrate the robot spin speed
# ------------------------------------------------------------------------------------------------------
def calibrateSpins(spins, seconds):
    #global _calibratedSpins
    global _calibratedSpinsPerSecond

    _calibratedSpinsPerSecond = spins / seconds
    #_calibratedSpins = True

# ------------------------------------------------------------------------------------------------------
# Calculate the wheel circumference
# ------------------------------------------------------------------------------------------------------
def wheelCircumferenceInMillimetres():
    return math.pi*wheelDiameterInMillimetres

# ------------------------------------------------------------------------------------------------------
# Calculate the theoretical max speed based on the wheel circumference and motor speed
# ------------------------------------------------------------------------------------------------------
def theoreticalSpeedInMillimetresPersSecond():
    return wheelCircumferenceInMillimetres() * motorMaxSpeedRPM / 60

# ------------------------------------------------------------------------------------------------------
# Automatically calibrate the robot based on wheel diameter, wheel spacing and motor rpm
# ------------------------------------------------------------------------------------------------------
def autoCalibrateSpeed():
    # Actual speed is roughly 0.72 times theoretical speed for the motors we use
    calibrateSpeedMillimetresPerSecond(theoreticalSpeedInMillimetresPersSecond() * 0.72)



# ======================================================================================================
# Define Public Functions: Helper Functions
# ======================================================================================================

# ------------------------------------------------------------------------------------------------------
# Wait for a specific time period
# ------------------------------------------------------------------------------------------------------
def wait(seconds):
    time.sleep(seconds)

# ------------------------------------------------------------------------------------------------------
# Wait for Enter to be pressed
# ------------------------------------------------------------------------------------------------------
def waitForKey():
    input("Press Enter")

# ------------------------------------------------------------------------------------------------------
# Turn tracing messages on
# ------------------------------------------------------------------------------------------------------
def tracingOn():
    global _tracing
    _tracing = True

# ------------------------------------------------------------------------------------------------------
# Turn tracing messages off
# ------------------------------------------------------------------------------------------------------
def tracingOff():
    global _tracing
    _tracing = False

# ------------------------------------------------------------------------------------------------------
# Set settings from our standard robots
# ------------------------------------------------------------------------------------------------------
def settings(name):
    global wheelDiameterInMillimetres
    global wheelSpacingInMillimetres
    global motorMaxSpeedRPM
    global calibratedSpinsPerSecond
    global robotName

    robotName = name
    if name.lower()=="floella":
        wheelDiameterInMillimetres = 60
        wheelSpacingInMillimetres = 123
        motorMaxSpeedRPM = 240
        #calibratedSpinsPerSecond = 0.959693
        calibrateSpeed()
        calibrateSpins(52, 60)        
    elif name.lower()=="gertrude":
        wheelDiameterInMillimetres = 42
        wheelSpacingInMillimetres = 140
        motorMaxSpeedRPM = 240
        calibrateSpeed(millimetres=2090, seconds=5)
        calibrateSpins(spins=40,seconds=30) 
    elif name.lower()=="edwina":
        wheelDiameterInMillimetres = 32
        wheelSpacingInMillimetres = 114
        motorMaxSpeedRPM = 80
        calibrateSpeed(millimetres=1110, seconds=10)
        calibrateSpins(spins=19.8, seconds=60)
    else:
        raise RosiException("Oops!  I don't know about a robot called {}".format(robotName))

# ------------------------------------------------------------------------------------------------------
# Show the robot settings
# ------------------------------------------------------------------------------------------------------
def printSettings():
    print ("Robot Settings:")
    print ("-----------------------------------------------------------")
    print ("Robot name ", robotName)
    print ("Wheel diameter ", wheelDiameterInMillimetres)
    print ("Motor max speed", motorMaxSpeedRPM)
    print ("Wheel circumference ", round(wheelCircumferenceInMillimetres(),2))
    print ("Theoretical Speed in Millimetre Per Second ",  round(theoreticalSpeedInMillimetresPersSecond(),2))
    print ("Calibrated Speed in Millimetres Per Second", _calibratedSpeedMillimetresPerSecond)
    print ("Calibrated Spins per Second", round(_calibratedSpinsPerSecond,2))
    print ("\n")

# ------------------------------------------------------------------------------------------------------
# Get all the command line arguments
# ------------------------------------------------------------------------------------------------------
def getCommandArguments():
    return sys.argv[1:]

# ------------------------------------------------------------------------------------------------------
# Get an int command line argument
# ------------------------------------------------------------------------------------------------------
def getIntArgument(number, default):
    try:
        if len(sys.argv) < number+2:
            rv = default
        else:
            rv =  int(sys.argv[number-1])
    except ValueError:
        raise RosiException("Oops!  You provided the argument '{}' which couldn't be converted to an integer".format(sys.argv[number-1]))
    else:
        return rv

# ------------------------------------------------------------------------------------------------------
# Get a string command line argument
# ------------------------------------------------------------------------------------------------------
def getStringArgument(number, default):
    if len(sys.argv) < number+2:
        rv = default
    else:
        rv =  sys.argv[number-1]
    return sys.argv[number-1]

# ------------------------------------------------------------------------------------------------------
# Get a float command line argument
# ------------------------------------------------------------------------------------------------------
def getFloatArgument(number, default):
    try:
        if len(sys.argv) < number+2:
            rv = default
        else:
            rv =  float(sys.argv[number-1])
    except ValueError:
        raise RosiException("Oops!  You provided the argument '{}' which couldn't be converted to a float".format(sys.argv[number-1]))
    else:
        return rv



