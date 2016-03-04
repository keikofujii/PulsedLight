"""
garys_test_user_to_user.py

A module to test and see if, when the drone is armed by the user, the drone can take over at a specified
altitude, then execute a command (in this case, fly to Austraila for a bit), then give the control back 
to the user in the mode that the drone was originally in. 

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

# We will need to store the initial mode so that the drone will know which mode to go back to
previousMode = None


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

    # The user will take off and start the drone going up to a specific altitude
    # Wait until the vehicle is just below target altitude
    while not (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)

    print "Reached target altitude"

    # store the previous mode so we can switch back into it later
    previousMode = vehicle.mode.name

    # Switch into GUIDED mode
    print "Going from %s mode into GUIDED mode" % previousMode
    vehicle.mode    = VehicleMode("GUIDED")

arm_and_takeoff(2)

print "Set default/target airspeed to 1"
vehicle.airspeed = 1

print "Going towards first point for 5 seconds ..."
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change
time.sleep(5)

# Go back to the previous mode
print "Going to previous mode: %s" % previousMode
vehicle.mode = VehicleMode(previousMode)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
