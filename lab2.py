#!/usr/bin/env python3
'''COM2009-3009 EV3DEV TEST PROGRAM'''

# Connect left motor to Output C and right motor to Output B
# Connect an ultrasonic sensor to Input 3

import os
import sys
import time
import ev3dev.ev3 as ev3
import math as Math
import sys
from statistics import stdev
from statistics import mean

import time

# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12')

    # display something on the screen of the device
    print('Hello World!')

    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    # announce program start
    ev3.Sound.speak('PENIS').wait()

    # set the motor variables
    mb = ev3.LargeMotor('outB') # b is left
    mc = ev3.LargeMotor('outC') # c is right
    velL = 20
    velR = 10

    # set the ultrasonic sensor variable
    us3 = ev3.UltrasonicSensor('in3')
    us2 = ev3.UltrasonicSensor('in2')

    ###
    ### 
    ###

    # define constants
    Kp = .7
    Ki = .7 # .13 - calculation says 34
    Kd = .02 # 3 
    offset = 0
    targetDistance = 20
    maxPower = 100
    minPower = 55
    previousError = 0
    integralError = 0
    previousIntegralError = 0
    speed = 100
    dT = 1
    while True:
        # we're going to want to calculate the time
        start = time.time()

        # calculate the offset 
        leftOffset = us3.value() / 10
        rightOffset = us2.value() / 10

        # the error will be the difference between distance to left and right wall
        #error = offset - targetDistance
        error = rightOffset - leftOffset
        integralError = integralError + error #* dT
        derivativeError = error - previousError

        if(error * previousIntegralError < 0):
            integralError = 0
            
        # calculate the differential and left power and right power       
        differential = Kp * error + (Ki * integralError * dT) + (Kd * derivativeError / dT)
        leftPower = speed + differential
        rightPower = speed - differential
        
        #power = Kp * error + Ki * integralError  + Kd * derivativeError
        leftPower = leftPower if leftPower < maxPower else maxPower
        rightPower = rightPower if rightPower < maxPower else maxPower

        rightPower = minPower if rightPower < minPower else rightPower
        leftPower = minPower if leftPower < minPower else leftPower



        previousIntegralError = integralError
        #power = power if power < maxPower else maxPower
        #power = power if power > minPower else minPower

        mb.run_direct(duty_cycle_sp=leftPower)
        mc.run_direct(duty_cycle_sp=rightPower)

        debug_print('Left Distance: ', leftOffset )
        debug_print('Right distance ', rightOffset )
        debug_print('kp component ', Kp * error)
        debug_print('Integral Component ', integralError * Ki * dT)
        debug_print('Derivative Componenet ', derivativeError * Kd / dT)
        debug_print('Elapsed time ', dT)
        debug_print('left Power ', leftPower)
        debug_print('right Power ', rightPower)
        debug_print('differential  ', differential)
        debug_print(' ')

        end = time.time() # 2.5 in 2 seconds
                          # 2.5 in 2.4 seconds
                          # 4 in 2.58
                          # 4 in 2.66

                        # 6 in 4.2
                        # 6 in 4.3
                                        
        
        dT = end - start
        #time.sleep(.05)
            
        # display the distance on the screen of the device
        #print('Distance =',ds)

        # print the distance to the output panel in VS Code

        # move
        #mb.run_direct(duty_cycle_sp=Math.fabs(velL - velR))
        #mc.run_direct(duty_cycle_sp=-Math.fabs(velL - velR))
        

        # stop
        #mb.run_direct(duty_cycle_sp=0)
        #mc.run_direct(duty_cycle_sp=0)
        
        # reverse direction
        #sp = -sp
    
    # announce program end
    ev3.Sound.speak('Test program ending').wait()

def sameSign(num1, num2):
    return (num1 < 0 and num2 < 0) or (num1 > 0 and num2 > 0) or (num1 == num2)

if __name__ == '__main__':
    main()