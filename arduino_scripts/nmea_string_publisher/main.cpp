/*

This script will read is serial data from 
the Adafruit GPS breakout board to the 
Arduino Mega's Serial3 port. 

The GPS board will be set to only output
two types of NMEA sentences; GPGGA, and 
GPRMC.  The sentences will each be saved 
to a ROS String (std_msgs/String.h) 
anytime a new sentence is recieved from 
the GPS board.

The ROS String is published on the 'raw_nmea'
topic.

*/

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "GPS.h"

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher raw_gprmc("raw_gprmc", &str_msg);
ros::Publisher raw_gpgga("raw_gpgga", &str_msg);

//char hello[13] = "hello world!";
String message;


// setup
void setup()
{
	init();
	Serial.begin(57600);
	// Serial.println("Program started...");
	// Serial.println("---");
	delay(1000);

	nh.initNode();
  nh.advertise(raw_gprmc);
  nh.advertise(raw_gpgga);
}


// main
int main()
{
	// setup timers, UART, etc.
	setup();

	// create a gps object
	GPS gps;

	while(true)
	{
		// update gps data
		gps.get_nmea_sentences();
		if(gps.updated)
		{
			// Serial.println(gps.gpgga);
			// Serial.flush();
			// Serial.println(gps.gprmc);
			// Serial.flush();
			// Serial.println("---");
			// Serial.flush();

			// publish GPRMC data
			message = gps.gprmc;
			str_msg.data = message.c_str();
		  raw_gprmc.publish( &str_msg );

		  // publish GPGGA data
		  message = gps.gpgga;
			str_msg.data = message.c_str();
		  raw_gpgga.publish( &str_msg );

		  // process publishers
		  nh.spinOnce();
		}
	}
}


/* 

To view the NMEA sentences from Laptop running ROS, 
run the following commands:

	roscore
	rosrun rosserial_python serial_node.py /dev/ttyACM4 _baud:=57600
	rostopic echo raw_gpgga
	rostopic echo raw_gprmc


Output should look like this:

	data: "$GPGGA,233559.000,3342.7327,N,09635.6895,W,1,05,1.59,198.2,M,-24.7,M,,*5F\r"
	---


	data: "$GPRMC,233558.000,A,3342.7327,N,09635.6894,W,0.48,264.87,170419,,,A*72\r"
	---

*/






/* 

while(true)
	{
		// update gps data
		gps.get_nmea_sentences();
		if(gps.updated)
		{
			Serial.println(gps.gpgga);
			Serial.flush();
			Serial.println(gps.gprmc);
			Serial.flush();
			Serial.println("---");
			Serial.flush();
		}
	}


OUTPUT - With satellite fix:


OUTPUT - Without satellite:

	$GPGGA,233653.999,,,,,0,00,,,M,,M,,*73
	$GPRMC,233653.999,V,,,,,0.00,0.00,160419,,,N*4D


*/






