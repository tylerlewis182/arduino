#include <Arduino.h>
#include "GPS.h"


// setup
void setup()
{
	init();
	Serial.begin(115200);
	Serial.println("Program started...");
	Serial.println("---");
	delay(1000);
}




// main
int main()
{
	// setup timers and serial
	setup();

	// create a gps object
	GPS gps;

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

}
