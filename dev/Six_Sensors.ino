/******************************************************************************
*
* Name        : Denis Yablonsky and Bryan Neufeld
* Program     : Six Sensor
* Date        : 9-19-15
* Description : This program runs 6 Pulse Light LIDARLite v2 sensors. It
*               calibrates them and then reads distances from them.
*
******************************************************************************/

//-----------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//-----------------------------------------------------------------------------

#include <Wire.h>
#include <LIDARLite.h>

//-----------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//-----------------------------------------------------------------------------

#define NUMBER_OF_SENSORS 5
#define OFFSET_REGISTER 0x13

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

// variable for holding the command to read a sensor
char command;


//==============================================================================
// The setup section sets the I2C addresses of the 6 sensors and then calibrates
// each sensor according to its individual offset.
//==============================================================================
void setup()
{
  // start the Arduino serial monitor
  Serial.begin(115200);

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
  myLidarLite.write(OFFSET_REGISTER, 0xF4, addresses[4]);
  delay(10);
  //myLidarLite.write(OFFSET_REGISTER, 0x00, addresses[5]);
  //delay(10);
  
} // setup


//==============================================================================
// The loop section reads constantly from each sensor
//==============================================================================
void loop()
{
  // Wait for a serial command to determine which sensor to read
  if (Serial.available())
  {
    // read the character that was sent over
    command = Serial.read();

    // based on the character read the desired sensor
    switch (command)
    {
      case '1':
        Serial.print(myLidarLite.distance(true, true, addresses[0]));
        break;
      case '2':
        Serial.print(myLidarLite.distance(true, true, addresses[1]));
        break;
      case '3':
        Serial.print(myLidarLite.distance(true, true, addresses[2]));
        break;
      case '4':
        Serial.print(myLidarLite.distance(true, true, addresses[3]));
        break;
      case '5':
        Serial.print(myLidarLite.distance(true, true, addresses[4]));
        break;
      /*case '6':
        Serial.print("Sensor 6: ");
        Serial.print(myLidarLite.distance(true, true, addresses[5]));
        break;*/
      default:
        Serial.print("-1");
    } // switch on command

    Serial.print("\n");

    // delay between sensor readings for accuracy
    delay(1);

  } // if serial available

} // loop


