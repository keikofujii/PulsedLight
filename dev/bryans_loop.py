"""
Modification of simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse  
from pymavlink import mavutil

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    print "Send velocity for 5 seconds"
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
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

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
            break
        time.sleep(1)

def main():
    """
    Main function to set a drone's position
    """ 
    # Declarations for setting drone position target local ned
    # Set up velocity mappings
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend
    # duration is in seconds?
    VELOCITY_X = 4 # Go North
    VELOCITY_Y = 0 # Do not go East or West
    VELOCITY_Z = 0 # Do not go up or down
    DURATION = 10 

    # //* Moved this from the top 
    # Set up option parsing to get connection string
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='127.0.0.1:14550',
                       help="vehicle connection target. Default '127.0.0.1:14550'")
    args = parser.parse_args()


    # Connect to the Vehicle
    print 'Connecting to vehicle on: %s' % args.connect
    vehicle = connect(args.connect, wait_ready=True)
    # //* Moved code ends here

    arm_and_takeoff(vehicle, 1) # //* Bryan changed from 10 to 1 for testing

    # print "Set default/target airspeed to 3"
    # vehicle.airspeed=3

    # print "Going towards first point for 30 seconds ..."
    # point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
    # vehicle.simple_goto(point1) /** KF

    # send command to vehicle to set position target local ned
    send_ned_velocity(vehicle, VELOCITY_X, VELOCITY_Y, VELOCITY_Z, DURATION)

    # sleep so we can see the change in map
    # time.sleep(30)

    # print "Going towards second point for 30 seconds (groundspeed set to 10 m/s) ..."
    # point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
    # vehicle.simple_goto(point2, groundspeed=10)

    # sleep so we can see the change in map
    # time.sleep(30)

    print "Returning to Launch"
    vehicle.mode    = VehicleMode("RTL")

    #Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()

if __name__ == "__main__":
    main()
