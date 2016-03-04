"""
garys_test_full_auto.py

A module to test and see if the drone can be armed, switched to GUIDED mode, moved, and landed automatically
with no user input at all.

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

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle is just below target altitude
    while not (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)

    print "Reached target altitude"

arm_and_takeoff(2)

# Set the vehicle airspeed 
print "Set default/target airspeed to 1"
vehicle.airspeed = 1

# Move toward a point that is given in GPS coordinates
print "Going towards first point for 5 seconds ..."

# Currently, we are flying ot Austrailia. Why wouldn't you fly to Austrailia? 
# Austrailia has koalas. And dingos. And kiwis.
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change
time.sleep(5)

# Land the drone 
print "Landing"
vehicle._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                                  0, 0, 0, 0, 0, 0, 0, 0)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
