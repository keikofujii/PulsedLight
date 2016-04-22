"""
garys_test.py

A module to test and see if the drone can be armed, switched to GUIDED mode, moved, and 
then give the control back to the user in LOITER mode

"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to Raspberry Pi by default.')

# This connection string defaults to the connection string for the Raspberry Pi
parser.add_argument('--connect', default='/dev/ttyACM0',
                   help="vehicle connection target. Default '/dev/ttyACM0'")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    A method to arm the vehicle and to take off.  The drone will fly to a specified altitude.

    Params: 
    aTargetAltitude - An int that represents the altitude to fly to
    """

    print "Basic pre-arm checks"

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    # The user will take off and go to a specified altitude 

    # Wait until the vehicle is just below target altitude
    while not (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)

    print "Reached target altitude"

    # Switch into GUIDED mode
    print "Going into guided mode" 
    vehicle.mode    = VehicleMode("GUIDED")


arm_and_takeoff(2)

# Set the airspeed of the drone
print "Set default/target airspeed to 1"
vehicle.airspeed = 1

# Go toward Austrail for a bit
print "Going towards first point for 5 seconds ..."
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change (this sets how long you go towards a point)
time.sleep(5)

# Switch into the user mode LOITER
print "Going to LOITER"
vehicle.mode = VehicleMode("LOITER")

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
