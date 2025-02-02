# CamJam Edukit 3 - Robots
# Worksheet 9 - Obstacle Avoidance


# load the libraries
import RPi.GPIO as GPIO
import time
import numpy as np
from pirobot.motor_control import *
# Set GPIO pins for the ultrasonic sensor
pinTrigger = 17
pinEcho = 18

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

def measure_distance():
    # Send pulse to trigger
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)

    start_time = time.time()
    stop_time = time.time()

    # Waiting for the Echo to go high
    while GPIO.input(pinEcho) == 0:
        start_time = time.time()

    # Waiting for Echo to go low
    while GPIO.input(pinEcho) == 1:
        stop_time = time.time()

    # Calculate distance
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Distance in cm
    return distance


# Distance variables
hownear = 40.0
reversetime = 0.5
turntime = 0.75

def initialize_sensors():
    #set GPIO modes
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Enable Ultrasonic sensors
    GPIO.setup(pinTrigger, GPIO.OUT)
    GPIO.setup(pinEcho, GPIO.IN)
    GPIO.output(pinTrigger, False)
    time.sleep(0.1)

# global variable needed for obstacle avoidance
left = False 

def measure():
    #  Average over 5 readings to reduce noise
    dist = []
    for i in range(5):
        #set trigger to False
        GPIO.output(pinTrigger, False)
        time.sleep(0.0001)

        # send out a pulse at a freq of 10 micro hertz
        GPIO.output(pinTrigger, True)
        time.sleep(0.00001)
        GPIO.output(pinTrigger, False)

        # start the timer
        StartTime = time.time()

        # start time is reset until the Echo Pin is taken high
        while GPIO.input(pinEcho) == 0:
            StartTime = time.time()
            
        # stop when echo pin is no longer high - end time
        while GPIO.input(pinEcho) == 1:
            StopTime = time.time()
            interval = StopTime - StartTime
            if interval >= 0.02:
                #print('Either you are too close or too far to me to see.')
                #StopTime = StartTime
                break
        # calculate pulse length
        ElapsedTime = StopTime - StartTime

        # Distance travelled by the pulse in that time in cm
        Distance = (ElapsedTime * 34326)/2.0
        dist.append(Distance)
    return np.mean(dist) 
    
def is_near_obstacle(threshold=15.0):
    # Example code for obstacle detection
    distance = measure_distance()  # Assuming you have a measure_distance function
    return distance < threshold


def avoidobstacle():
    global left
    backward() 
    time.sleep(reversetime)
    stopmotors()
    if left:
        turnright()
        left = False
    else:
        turnleft()
        left = True
    time.sleep(turntime)
    stopmotors()



def halt():
    stopmotors()

def initialize():
    initialize_motors()
    initialize_sensors()
    
def main():
    if nearobstacle(hownear):
        stopmotors()
        avoidobstacle()
        msg = 'Turning to avoid obstacle'
        return msg
    else:
        forward()
        time.sleep(0.1)
        msg = 'Moving forward'
        return msg


############
if __name__ == '__main__':
    try:
        GPIO.output(pinTrigger, False)
        # Allow module to settle
        time.sleep(0.1)
        left = False
        while True:
            forward()
            time.sleep(0.1)
            if nearobstacle(hownear):
                stopmotors()
                left = avoidobstacle(left)
    except KeyboardInterrupt:
        GPIO.cleanup()
    
        
