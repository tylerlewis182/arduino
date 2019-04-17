#include <Arduino.h>
#include "GPS.h"
#include "nmea.h"



// setup
void setup()
{
	init();
	Serial.begin(115200);
	Serial.println("Program started...");
	Serial.println("---");
	delay(1000);
}

// timers
unsigned long start_time = 0;
unsigned long total_time = 0;
unsigned long t0 = 0;
unsigned long t1 = 0;



// main
int main()
{
	// setup timers and serial
	setup();

	// create a gps object
	GPS gps;

	// create nmea sentence objects for GPRMC and GPGGA
	NMEA gpgga;
	NMEA gprmc;

	while(true)
	{

		//start_time = millis();                      
		// update gps data if serial data is available
		gps.get_nmea_sentences(); // 75 MILLIseconds
		//total_time = millis() - start_time;


		// if both GPGGA and GPRMC sentences have been updated, update the two nmea objects
		if(gps.updated) // 1 second (without printing)
		{
			gprmc.s = gps.gprmc;
			
			Serial.println(gps.gprmc);
			Serial.flush();
			Serial.println(gprmc.s);
			Serial.flush();


			bool ret = gprmc.check_validity();
			Serial.println(ret);
			Serial.flush();
			Serial.println(gprmc.valid);
			Serial.flush();

			




			ret = gprmc.checksum();
			Serial.println(ret);
			Serial.flush();


			




			
			print("---");
			
			//gpgga.update(gps.gpgga); // TODO: finish update() in nmea.h 
			//gprmc.update(gps.gprmc); // 500 microseconds
			
			
			print("---");
			//print(gprmc.provided_chk_hex);
			print("---");


			
		}





	}
}


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






