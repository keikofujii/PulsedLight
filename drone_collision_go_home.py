"""
drone_collision_detection.py

This module describes a drone collision detection system.  
It will let the user initialize and arm the drone, then will run two threads in 
the background.  
The first thread will populate a queue that serves as a buffer that holds the most 
recent relative GPS locations.
The second will wait for sensor input, and if it receives sensor input, then it will
go into GUIDED mode, and move the drone toward the last two previous locations in order
to back away from an obstacle. Control will then be returned to the user in the previous 
mode.

"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import threading
from collections import deque
import subprocess
import RPi.GPIO as GPIO
from lidarLite import Lidar_Lite
import os.path
import threading
import math

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to Raspberry Pi by default.')

#The range that we consider a collision. It's in centimeters, so 100 cm means
#that we'll consider something a collision if it's 1 meter away from us. 
global COLLISION_BUFFER
COLLISION_BUFFER = 100
global NUMBER_OF_SENSORS
NUMBER_OF_SENSORS = 1
global GPIO_NUMBERS
GPIO_NUMBERS = [17, 18, 27, 22, 23, 24, 25, 4]
global gpsHistoryQueue
global vehicle
global previousMode
global distanceReadings
global HORIZONTAL__COLLISION_THRESHOLD
global VERTICAL_COLLISION_THRESHOLD
global HORIZONTAL_RECORDING_THRESHOLD
global VERTICAL_RECORDING_THRESHOLD
HORIZONTAL_COLLISION_THRESHOLD = 1.5
VERTICAL_COLLISION_THRESHOLD = 1.5
HORIZONTAL_RECORDING_THRESHOLD = 3
VERTICAL_RECORDING_THRESHOLD = 3
BOTTOM_SENSOR_THRESHOLD = 75
global QUEUE_LENGTH
QUEUE_LENGTH = 1024

def initialize_and_arm():
    """
    A method to check that the vehicle is initialized and armed.
    """

    print "Basic pre-arm checks"
    # check the autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    # Wait until the vehicle is armed
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Vehicle armed."

def withinThreshold(currentGPS, targetGPS, horizontalThreshold, verticalThreshold):
        latDistance = currentGPS.lat - targetGPS.lat
        longDistance = currentGPS.lon - targetGPS.lon

        horizontalDistance = math.sqrt(latDistance*latDistance + longDistance*longDistance) * 1.113195e5

        return horizontalDistance < horizontalThreshold and math.fabs(currentGPS.alt - targetGPS.alt) < verticalThreshold

class SensorThread(object):
    """ Sensor Thread class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self):#, interval=1): //* interval may not be needed
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        #self.interval = interval //* interval may not be needed

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def hazCollision(self, lidar):
        """
        This module parses the data that comes back from the C code that is
        started in the run() method.
        """
        
        distanceReadings = [-1]*NUMBER_OF_SENSORS#array to hold current distance values

        #iterate over all sensors and get thie values
        for i in range(0, NUMBER_OF_SENSORS):
            #Turn sensor on
            GPIO.output(GPIO_NUMBERS[i], True)
            time.sleep(.02)#wait for it to turn on
            
            #Get reading
            distanceReadings[i] = lidar.getDistance()

            print(distanceReadings[i])
            
            #Turn sensor off
            GPIO.output(GPIO_NUMBERS[i], False)
        collision = False

        i = 0 
        # Iterate through all the sensors and see if any of them see an obstacle
        # that is too close to the drone 
        while (i < len(distanceReadings) and not collision):
            if distanceReadings[i] > BOTTOM_SENSOR_THRESHOLD and distanceReadings[i] < vehicle.airspeed*.08*NUMBER_OF_SENSORS + COLLISION_BUFFER:
                collision = True
            i += 1
        
        return collision        
        

    def run(self):
        """ Method that runs forever """
        # Get the home postition
        home = vehicle.location.global_relative_frame
	
	home.alt = home.alt + 2
	print home        
        lidar = Lidar_Lite()

        #Attempt to connect to the lidar sensor(s)
        connected = lidar.connect(1)

        #If not able to connect then print command
        if connected < -1:
          print "Not Connected"#CS people do what you want here

        #turn off gpio warnings
        GPIO.setwarnings(False)

        #Set GPIO enumeration Type (Pi vs Board)
        GPIO.setmode(GPIO.BCM)

        #turn off all sensors by pulling GPIO pins to 0
        for i in range(0, NUMBER_OF_SENSORS):
            GPIO.setup(GPIO_NUMBERS[i], GPIO.OUT)#setting gpio mode to output
            GPIO.output(GPIO_NUMBERS[i], False)#set gpio pin to 0

        # Infinite loop
        while True:
            
            # This line is sensor code
            if self.hazCollision(lidar):

                # store the previous mode so we can switch back into it later
                previousMode = vehicle.mode.name

                # Switch into GUIDED mode
                print "Going from %s mode into GUIDED mode" % previousMode
                vehicle.mode = VehicleMode("GUIDED")

                # Set the airspeed to 2
                print "Set target airspeed to 2"
                vehicle.airspeed = 2

                # Go toward the previous GPS location
                print "Going towards home"

                # Go toward home for 2 seconds
                vehicle.simple_goto(home)
                time.sleep(2)

                # Go back to the previous mode
                print "Going to previous mode: %s" % previousMode
                vehicle.mode = VehicleMode(previousMode)


# This connection string defaults to the connection string for the Raspberry Pi
parser.add_argument('--connect', default='/dev/ttyACM0',
                   help="vehicle connection target. Default '/dev/ttyACM0'")
args = parser.parse_args()

# Wait until the USB cable is plugged in
print "Waiting for USB"
while not os.path.exists(args.connect):
    pass

# Connect to the Vehicle
# This will timeout after 10 minutes (600 seconds). The default is 30 seconds. 
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=600)

# Make sure the vehichle is initialized
initialize_and_arm()

# We will need to store the initial mode so that the drone will know which mode to go back to
previousMode = None

# Starts the thread that will watch for sensor input
sensorThread = SensorThread()

# Vehicle object will never be closed
while True:
	pass
