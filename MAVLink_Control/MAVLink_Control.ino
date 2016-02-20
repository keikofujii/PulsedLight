/******************************************************************************
*
* Name        : Denis Yablonsky and Bryan Neufeld
* Program     : MAVLink_Control
* Date        : 1-18-16
* Description : This program controls the UAV with MAVLink.
*
******************************************************************************/

//-----------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//-----------------------------------------------------------------------------

// includes needed for MAVLink communication
//* I don't know if fast serial is needed anymore
//* #include <FastSerial.h>
#include <get_pixhawk.h>

//-----------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//-----------------------------------------------------------------------------

#define DELAY_BETWEEN_HEARTBEATS 1000 // 1 second

// Struct containing information on the MAV we are currently connected to

struct Drone_Data
{
  uint8_t sysid;
  uint8_t compid;
  mavlink_heartbeat_t heartbeat;
  //* don't know if we need to sync clocks
  //* uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
  float x; /*< X Position*/
  float y; /*< Y Position*/
  float z; /*< Z Position*/
  float yaw; /*< Yaw angle (rad, -pi..+pi)*/
};

//-----------------------------------------------------------------------------
//                __          __        ___  __
//     \  /  /\  |__) |  /\  |__) |    |__  /__`
//      \/  /~~\ |  \ | /~~\ |__) |___ |___ .__/
//
//-----------------------------------------------------------------------------

// Serial needed for MAVLink to communicate
//* FastSerialPort0(Serial);

// MAVLink system definitions (these were serial sniffed from MissionPlanner)
/*uint8_t sysid = 255; // ID for this UAV
uint8_t compid = MAV_COMP_ID_UART_BRIDGE; // The component sending the message
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
uint8_t system_mode = 0; // Completely undefined
uint32_t custom_mode = 0x00080600; // Custom mode shows up as 0x00060800
uint8_t system_state = MAV_STATE_UNINIT;
uint8_t target_sys = 1;
uint8_t target_comp = 1;*/ //* making all these #defines for space

#define sysid 255 // ID for this UAV
#define compid MAV_COMP_ID_UART_BRIDGE // The component sending the message
#define system_type MAV_TYPE_GENERIC
#define autopilot_type MAV_AUTOPILOT_GENERIC
#define system_mode 0 // Completely undefined
#define custom_mode 0x00080600 // Custom mode shows up as 0x00060800
#define system_state MAV_STATE_UNINIT
#define target_sys 1
#define target_comp 1

// Initialize the required buffers for MAVLink
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;

// store some data about the drone's current state
Drone_Data current_drone_data;

// required booleans to establish local position
uint8_t got_position = false;
uint8_t got_yaw = false;

//==============================================================================
// The setup section starts the serial communication required to communicate
// MAVLink commands. It also sends some start up commands.
//==============================================================================
void setup()
{
  // start serial communication for MAVLink at the Pixhawk's prefered baud rate
  Serial.begin(57600); // some tutorials say 921600 which is wicked fast

  /*mavlink_msg_request_data_stream_pack_chan( sysid,  compid,  MAVLINK_COMM_0, &msg,
                                              target_sys,  target_comp, MAV_DATA_STREAM_ALL, 10, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/

  /*mavlink_msg_command_long_pack(sysid, compid, &msg, 1,
                                1, MAV_CMD_DO_SET_MODE, 1,
                                6, 0, 0, 0, 0, 0, 0); // PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/

  while (!(got_position && got_yaw))
  {
     comm_receive();
  }


for ( int i = 0; i < 16; i++)
  {
    mavlink_msg_set_position_target_local_ned_pack(sysid, compid, &msg,
        millis(), target_sys, target_comp, MAV_FRAME_LOCAL_NED, 0b0000100111000111, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    delay(250);
  }

  // enable offboard
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_sys,
                                target_comp, MAV_CMD_NAV_GUIDED_ENABLE, 1,
                                1, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  // give the command some time to sink in
  //delayMicroseconds(100);
  delay(250);

  // send set points // trying to change velocity

  for ( int i = 0; i < 32; i++)
  {
    mavlink_msg_set_position_target_local_ned_pack(sysid, compid, &msg,
        millis(), target_sys, target_comp, MAV_FRAME_LOCAL_NED, 0b0000100111000111, 0, 0, 0,
        2.0 , 0, 0, 0, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    delay(250);
  }

  // disable offboard
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_sys,
                                1, MAV_CMD_NAV_GUIDED_ENABLE, 1,
                                0, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);



} // setup


//==============================================================================
// The loop section sends MAVLink commands constatnly
//==============================================================================
void loop()
{

} // loop


//==============================================================================
// This function recieves MAVLink packets from the UAV.
//==============================================================================
void comm_receive()
{
  mavlink_status_t status;

  while (Serial.available() > 0)
  {
    uint8_t c = Serial.read();
    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      // Handle message
      switch (msg.msgid)
      {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
            mavlink_msg_heartbeat_decode(&msg, &(current_drone_data.heartbeat));
            break;
          }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
          {
            //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
            current_drone_data.x = mavlink_msg_local_position_ned_get_x(&msg);
            current_drone_data.y = mavlink_msg_local_position_ned_get_y(&msg);
            current_drone_data.z = mavlink_msg_local_position_ned_get_z(&msg);
            got_position = true;
            break;
          }
        case MAVLINK_MSG_ID_ATTITUDE:
          {
            //printf("MAVLINK_MSG_ID_ATTITUDE\n");
            current_drone_data.yaw = mavlink_msg_attitude_get_yaw(&msg);
            got_yaw = true;
            break;
          }
        default:
          {
            // printf("Warning, did not handle message id %i\n",message.msgid);
            break;
          }
      }
    }
  }
}


