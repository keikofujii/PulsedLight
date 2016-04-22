/******************************************************************************
*
* Name        : Denis Yablonsky and Bryan Neufeld
* Program     : LIDAR_Control
* Date        : 12-6-25
* Description : This program runs the LIDAR Sensors
*
******************************************************************************/

//-----------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//-----------------------------------------------------------------------------

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
#define TRIGGER_DISTANCE 100 // 1 meters
#define SPEAKER_PIN 10
#define SPEAKER_FREQUENCY 440 // tuning A

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

// keep a variable for the shortest distance
int shortest_distance;

//==============================================================================
// The setup section sets the I2C addresses of the 6 sensors and then calibrates
// each sensor according to its individual offset.
//==============================================================================
void setup()
{
  // start serial communication for MAVLink at the Pixhawk's prefered baud rate
  Serial.begin(57600); // 921600 which is wicked fast

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
  myLidarLite.write(OFFSET_REGISTER, 0xFC, addresses[2]);
  delay(10);
  myLidarLite.write(OFFSET_REGISTER, 0xFB, addresses[3]);
  delay(10);
  /*myLidarLite.write(OFFSET_REGISTER, 0xF4, addresses[4]);
  delay(10);*/
  //myLidarLite.write(OFFSET_REGISTER, 0x00, addresses[5]);
  //delay(10);

  // enable the watchdog timer
  //wdt_enable(WDTO_1S);
  Serial.println("\n-----reset-----");
} // setup


//==============================================================================
// The loop section reads constantly from each sensor and reacts if something
// gets too close.
//==============================================================================
void loop()
{
  // reset the watchdog timer
  //wdt_reset();

  // read each sensor and delay for a millisecond to help with accuracy
  for (int i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    distances[i] = myLidarLite.distance(true, true, addresses[i]);
    Serial.print(distances[i]);
    Serial.print("\t");
  }
  Serial.print("\n");


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
    // ring a buzzer
    tone(SPEAKER_PIN, SPEAKER_FREQUENCY);
  }
  else
  {
    noTone(SPEAKER_PIN);
  }

} // loop

