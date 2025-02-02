# motor control code


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np
import sys
from pirobot import key_press as kp
pinMotorAForward = 10
pinMotorABackward = 9
pinMotorBForward = 8
pinMotorBBackward = 7
pinTrigger = 17
pinEcho = 18

# PWM Parameters
Frequency = 20
DutyCycleA = 60
DutyCycleB = 60
Stop = 0

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the GPIO Pin Mode
GPIO.setup(pinMotorAForward, GPIO.OUT)
GPIO.setup(pinMotorABackward, GPIO.OUT)
GPIO.setup(pinMotorBForward, GPIO.OUT)
GPIO.setup(pinMotorBBackward, GPIO.OUT)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# Set the GPIO to Software PWM at Frequency Hertz
pwmMotorAForward = GPIO.PWM(pinMotorAForward, Frequency)
pwmMotorABackward = GPIO.PWM(pinMotorABackward, Frequency)
pwmMotorBForward = GPIO.PWM(pinMotorBForward, Frequency)
pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, Frequency)

# Start PWM with zero duty cycle (stopped)
pwmMotorAForward.start(Stop)
pwmMotorABackward.start(Stop)
pwmMotorBForward.start(Stop)
pwmMotorBBackward.start(Stop)



def stopmotors():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def forward():
    print('Moving Forward')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward():
    print('Moving backward')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
    print('Turning Left')
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
    print('Turning Right')
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)


def key_control():

    if kp.getKey('LEFT'):
        turnleft()

    elif kp.getKey('RIGHT'):
        turnright()
        
    elif kp.getKey('UP'):
        forward()
        
    elif kp.getKey('DOWN'):
        backward()

    elif kp.getKey('ESCAPE'):
        stopmotors()
        kp.stop()
        GPIO.cleanup()

    else:
        stopmotors()
        

def exec_cmd(cmd_str):
    if cmd_str == 'LEFT':
        print('Turning Left')
        turnleft()
    elif cmd_str == 'RIGHT':
        print('Turning Right')
        turnright()
    elif cmd_str == 'UP':
        print('Moving Forward')
        forward()
    elif cmd_str == 'DOWN':
        print('Moving Downward')
        backward()
    elif cmd_str == 'EXIT':
        print('Exiting')
        sys.exit()
    else:
        stopmotors()
        
    

if __name__ == '__main__':
    initialize_motors()
    stopmotors()
    kp.init()
    print('Controlling using keyboard')
    while True:
        try:
            key_control()
        except KeyboardInterrupt:
            GPIO.cleanup()
            kp.stop()
    


