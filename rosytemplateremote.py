#! /usr/bin/python3

# Think Create Learn
#
# ROSI remote control template
# Use a remote control to control a robot

# ======================================================================================================
# Imports
# ======================================================================================================
import rosi_library as robot
from approxeng.input.selectbinder import ControllerResource


# ======================================================================================================
# Constants
# ======================================================================================================


# ======================================================================================================
# Global variables
# ======================================================================================================


# ======================================================================================================
# Functions
# ======================================================================================================


# ======================================================================================================
# Main program
# ======================================================================================================
try:

    # Variable to indicate if we are still running
    running = True

    robot.start()

    with ControllerResource() as joystick:
        print ("Found joystick")
        while running and joystick.connected:

            presses = joystick.check_presses()
            
            if joystick.presses.start:
                # We want to exit the program                
                running = False
               
            if joystick.presses.cross:
                # ******** DO SOMETHING HERE ********
                print("Cross")
                
            if joystick.presses.dleft:
                # ******** DO SOMETHING HERE ********    
                print("D Left")           

            if joystick.presses.dright:
                # ******** DO SOMETHING HERE ********  
                print("D Right")             
                
            if joystick.ly:
                # ******** DO SOMETHING HERE ********    
                print("Stick Left Y ", joystick.ly) 
                               
            # Give the computer a bit of time to rest
            robot.wait(0.1)

    robot.finish()

except IOError:
    print("Unable to find any joystick")
except robot.RosiException as e:
    print(e.value)
except KeyboardInterrupt:
    robot.finish()    

