/******************************************************************************
*
* Name        : Denis Yablonsky and Bryan Neufeld
* Program     : LIDAR_Flight_Control
* Date        : 11-3-15
* Description : This program overrides control of a UAV to make sure it does
*               not collide with objects seen by LIDAR sensors. The program
*               runs 6 Pulse Light LIDARLite v2 sensors. It calibrates them and
*               then reads distances from them. It then sends out commands over
*               serial using the MAVLink protocol to stop the UAV from moving
*               closer to an object.
*
******************************************************************************/

//-----------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//-----------------------------------------------------------------------------

// includes needed for MAVLink communication
#include <FastSerial.h>
#include <../mavlink/include/mavlink.h>

// includes needed for LIDARLite sensors
#include <Wire.h>
#include <LIDARLite.h>

// include a watchdog timer for sensor failures
#include <avr/wdt.h>

//-----------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//-----------------------------------------------------------------------------

#define NUMBER_OF_SENSORS 4
#define OFFSET_REGISTER 0x13
#define TRIGGER_DISTANCE 100 // 3 meters
#define SPEAKER_PIN 8
#define SPEAKER_FREQUENCY 440 // tuning A
#define DELAY_BETWEEN_HEARTBEATS 1000 // 1 second

//-----------------------------------------------------------------------------
//                __          __        ___  __
//     \  /  /\  |__) |  /\  |__) |    |__  /__`
//      \/  /~~\ |  \ | /~~\ |__) |___ |___ .__/
//
//-----------------------------------------------------------------------------

// class for running LIDARLite functions
LIDARLite myLidarLite;

// Array of pins connected to the sensor Power Enable lines
int sensorPins[] = {2, 3, 4, 5, 6, 7};

// Array of I2C addresses for sensors
unsigned char addresses[] = {0x66, 0x68, 0x70, 0x72, 0x74, 0x76};

// Array of distance values being read
int distances[NUMBER_OF_SENSORS];

// Serial needed for MAVLink to communicate
FastSerialPort0(Serial);

// MAVLink system definitions (these were serial sniffed from MissionPlanner)
int sysid = 255; // ID for this UAV
int compid = MAV_COMP_ID_UART_BRIDGE; // The component sending the message
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
uint8_t system_mode = 0; // Completely undefined
uint32_t custom_mode = 0x00080600; // Custom mode shows up as 0x00060800
uint8_t system_state = MAV_STATE_UNINIT;


// Initialize the required buffers for MAVLink
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;

// keep a variable for the shortest distance
int shortest_distance;

// varible to keep track of time since the last heartbeat
unsigned long last_heartbeat;

//==============================================================================
// The setup section sets the I2C addresses of the 6 sensors and then calibrates
// each sensor according to its individual offset. It also starts the serial
// communication required to communicate MAVLink commands.
//==============================================================================
void setup()
{
  // start the Arduino serial monitor
  Serial.begin(57600);

  // initiate the LIDARLite class
  myLidarLite.begin();

  //set how many sensors, the pins they are connected to and their adresses
  myLidarLite.changeAddressMultiPwrEn(NUMBER_OF_SENSORS, sensorPins,
                                      addresses, false);
  delay(10);

  // calibrate all the sensors to their individual offsets based off average
  // readings. Do this by writing to control register 0x13 the offset value in 2s
  // compliment. These offsets get them pretty close (within 1 cm of each other)
  myLidarLite.write(OFFSET_REGISTER, 0xFD, addresses[0]);
  delay(10);
  myLidarLite.write(OFFSET_REGISTER, 0xFB, addresses[1]);
  delay(10);
  myLidarLite.write(OFFSET_REGISTER, 0xFB, addresses[2]);
  delay(10);
  myLidarLite.write(OFFSET_REGISTER, 0xFC, addresses[3]);
  delay(10);
  /*myLidarLite.write(OFFSET_REGISTER, 0xF4, addresses[4]);
  delay(10);*/
  //myLidarLite.write(OFFSET_REGISTER, 0x00, addresses[5]);
  //delay(10);

  // send a heartbeat for MAVLink connection
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type,
                             system_mode, custom_mode, system_state);Z
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  // reset the time since the last heartbeat
  last_heartbeat = millis();

  // enable the watchdog timer
  wdt_enable(WDTO_1S);
  //* Serial.println("\n-----reset-----");
} // setup


//==============================================================================
// The loop section reads constantly from each sensor
//==============================================================================
void loop()
{
  // reset the watchdog timer
  wdt_reset();

  // read each sensor and delay for a millisecond to help with accuracy
  for (int i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    distances[i] = myLidarLite.distance(true, true, addresses[i]);
    delay(100);
    //* Serial.print(distances[i]);
    //* Serial.print("\t");
  }
  //* Serial.print("\n");


  // find the shortest distance
  shortest_distance = distances[0];
  for (int i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    if (distances[i] < shortest_distance)
    {
      shortest_distance = distances[i];
    }
  }

  // check if the smalles distance read is within the trigger distance
  // if the distance is smaller than the trigger stop the UAV
  if (shortest_distance < TRIGGER_DISTANCE)
  {
    // send a stop command through MAVtLink
 //==============================================================================
    mavlink_msg_command_long_pack(sysid, compid, msg, target_system, 
                                  target_component, MAV_CMD_COMPONENT_ARM_DISARM, confirmation, 
                                  1, 0, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);

    // also ring a buzzer
    tone(SPEAKER_PIN, SPEAKER_FREQUENCY);
  }
  else
  {
    noTone(SPEAKER_PIN);
  }

  // at correct time send another heartbeat
  if ((millis() - last_heartbeat) >= DELAY_BETWEEN_HEARTBEATS)
  {
    // send a heartbeat
    mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type,
                               system_mode, custom_mode, system_state);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    // reset the time since the last heartbeat
    last_heartbeat = millis();
  }
  
  //comm_receive();
} // loop

void comm_receive() {
 
       mavlink_message_t msg;
	mavlink_status_t status;
 
	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
 
	while(Serial.available() > 0 ) 
	{
		uint8_t c = Serial.read();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
 
			switch(msg.msgid)
			{
			        case MAVLINK_MSG_ID_HEARTBEAT:
			        {
				  // E.g. read GCS heartbeat and go into
                                  // comm lost mode if timer times out
			        }
			        break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				// EXECUTE ACTION
				break;
			default:
				//Do nothing
				break;
			}
		}
 
		// And get the next one
	}
}


