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

authors: Keiko Fujii and Bryan Neufeld

"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import threading

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to Raspberry Pi by default.')

# This connection string defaults to the connection string for the Raspberry Pi
parser.add_argument('--connect', default='/dev/ttyACM0',
                   help="vehicle connection target. Default '/dev/ttyACM0'")
args = parser.parse_args()

# Connect to the Vehicle
# This will timeout after 10 minutes (600 seconds). The default is 30 seconds. 
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=600)

# We will need to store the initial mode so that the drone will know which mode to go back to
previousMode = None

# Needs a thread that will keep track of the last 10-20 GPS locations
gpsHistoryQueue = deque()

def arm_and_takeoff(aTargetAltitude):
    """
    A method to arm the vehicle and to take off.  The drone will fly to a specified altitude.

    Params: 
    aTargetAltitude - An int that represents the altitude to fly to
    """
    global previousMode

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    # Wait until the vehicle is armed
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Vehicle armed."

class GPSThread(object):
    """ GPS Thread class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, interval=1):
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        self.interval = interval

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        """ Method that runs forever """
        while True:
            global gpsHistoryQueue
            global vehicle

            # Want to continously populate GPS history
            while(true):
                # Limit the size of the queue to 10
                if (gpsHistoryQueue.qsize() == 10): 
                    # Add a new GPS location to the 'left hand side' of the deque
                    # So gpsHistoryQueue[0] will be the most recent location
                    # Not sure what form this will append this in
                    # I'm hoping for a list that contains [latitude, longitude, altitude]
                    gpsHistoryQueue.appendLeft(vehicle.location.global_relative_frame)

                    # Pop the least recent GPS location
                    gpsHistoryQueue.pop()
                else:
                    # Add a new GPS location to the 'left hand side' of the deque
                    # So gpsHistoryQueue[0] will be the most recent location
                    # Not sure what form this will append this in
                    # I'm hoping for a list that contains [latitude, longitude, altitude]
                    gpsHistoryQueue.appendLeft(vehicle.location.global_relative_frame)

                # Sleep for one second then get the new location
                time.sleep(1)

class SensorThread(object):
    """ Sensor Thread class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, interval=1):
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        self.interval = interval

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        """ Method that runs forever """
        while True:
            global gpsHistoryQueue
            global vehicle

            # Want to monitor the sensors and wait for input
            while(true):
                # This line is sensor code
                if sensorInput:
                    # store the previous mode so we can switch back into it later
                    previousMode = vehicle.mode.name

                    # Switch into GUIDED mode
                    print "Going from %s mode into GUIDED mode" % previousMode
                    vehicle.mode    = VehicleMode("GUIDED")

                    # Set the airspeed to 1
                    print "Set target airspeed to 1"
                    vehicle.airspeed = 1

                    # Go toward a previous GPS location for 2 seconds so that we get 2 meters away
                    print "Going towards previous GPS point for 2 seconds"

                    # Go toward the most recent point for 1 second
                    point1 = LocationGlobalRelative(gpsHistoryQueue[0][0], gpsHistoryQueue[0][1], gpsHistoryQueue[0][2])
                    vehicle.simple_goto(point1)

                    # sleep for one second so we can see a change
                    time.sleep(1)

                    # Go toward the second most recent point for 1 second, since points are gathered each second
                    point2 = LocationGlobalRelative(gpsHistoryQueue[1][0], gpsHistoryQueue[1][1], gpsHistoryQueue[1][2])
                    vehicle.simple_goto(point1)

                    # sleep for one second so we can see a change
                    time.sleep(1)

                    # Go back to the previous mode
                    print "Going to previous mode: %s" % previousMode
                    vehicle.mode = VehicleMode(previousMode)

# Starts the thread that will populate and keep track of the GPS locations
gpsThread = GPSThread()

# Starts the thread that will watch for sensor input
sensorThread = sensorThread()

arm_and_takeoff(2)

# Vehicle object will never be closed
