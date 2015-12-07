/******************************************************************************
*
* Name        : Denis Yablonsky and Bryan Neufeld
* Program     : MAVLink_Control
* Date        : 11-20-15
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

//-----------------------------------------------------------------------------
//                __          __        ___  __
//     \  /  /\  |__) |  /\  |__) |    |__  /__`
//      \/  /~~\ |  \ | /~~\ |__) |___ |___ .__/
//
//-----------------------------------------------------------------------------

// Serial needed for MAVLink to communicate
//* FastSerialPort0(Serial);

// MAVLink system definitions (these were serial sniffed from MissionPlanner)
int sysid = 255; // ID for this UAV
int compid = MAV_COMP_ID_UART_BRIDGE; // The component sending the message
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
uint8_t system_mode = 0; // Completely undefined
uint32_t custom_mode = 0x00080600; // Custom mode shows up as 0x00060800
uint8_t system_state = MAV_STATE_UNINIT;
uint8_t target_sys = 1;

// Initialize the required buffers for MAVLink
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;

// varible to keep track of time since the last heartbeat
unsigned long last_heartbeat;

//==============================================================================
// The setup section starts the serial communication required to communicate 
// MAVLink commands. It also sends some start up commands.
//==============================================================================
void setup()
{
  // start serial communication for MAVLink at the Pixhawk's prefered baud rate
  Serial.begin(57600); // 921600 which is wicked fast

  // send a heartbeat for MAVLink connection
  /*mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type,
                             system_mode, custom_mode, system_state);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  // reset the time since the last heartbeat
  //last_heartbeat = millis();
  
  // send an arm command to turn on the motors
  //  parameters: systemID, componentID, MAVLink message pointer, target system,
  //              target component, commandID, confirmation transmission count,
  //              arm/disarm, uneeded parameters
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_sys,
                                1, MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  delay(4000);
  
  // enable offboard
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_sys,
                                1, MAV_CMD_NAV_GUIDED_ENABLE, 1,
                                1, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  delay(100);
  
  mavlink_msg_set_position_target_local_ned_pack(sysid, compid, &msg,
                                                 millis(), target_sys, 1, MAV_FRAME_LOCAL_NED, 0b0000110111111000, -5.0, -5.0, 0,
                                                 0,0,0,0,0,0,0,0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
} // setup


//==============================================================================
// The loop section sends MAVLink commands constatnly
//==============================================================================
void loop()
{
  
  // at correct time send another heartbeat
  /*if ((millis() - last_heartbeat) >= DELAY_BETWEEN_HEARTBEATS)
  {
    // send a heartbeat
    /*mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type,
                               system_mode, custom_mode, system_state);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);*/
  // reset the time since the last heartbeat
  /*last_heartbeat = millis();
  }*/



  // try manual control
  // parameters: systemID, componentID, MAVLink message pointer, target system,
  //             pitch, roll, thrust, yaw, joystic buttons
  /*mavlink_msg_manual_control_pack(sysid, compid, &msg, target_sys,
                                  0, 0, 900, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/



  // send a comment to initialte thrust
  // parameters: systemID, componentID, MAVLink message pointer, target system,
  //             roll, pitch, yaw, thrust,
  //             roll_manual, pitch_manual, yaw_manual, thrust_manual
  /*mavlink_msg_attitude_control_pack(sysid, compid, &msg, target_sys,
                                    0, 0, 0, .9,
                                    1, 1, 1, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/



  /*const float q[4] = {1, 0, 0, 0};
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, millis(), target_sys, 1, 0x87, q,
                                       0, 0, 0, .9);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  
  
  /*mavlink_msg_set_position_target_local_ned_pack(sysid, compid, &msg,
                                                 millis(), target_sys, 1, MAV_FRAME_LOCAL_NED, 0b0000110111111000, -5.0, -5.0, 0,
                                                 0,0,0,0,0,0,0,0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/
  
  /*mavlink_msg_command_long_pack(sysid, compid, &msg, 1,
                                1, MAV_CMD_DO_SET_SERVO, 0,
                                1, 1999, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/




  // disarm
  /*mavlink_msg_command_long_pack(sysid, compid, &msg, 1,
                                1, MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                0, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);*/

  //comm_receive();
  
  delay(100);
} // loop


//==============================================================================
// This function recieves MAVLink packets from the UAV.
//==============================================================================
void comm_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available() > 0)
  {
    uint8_t c = Serial.read();
    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message

      switch (msg.msgid)
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


