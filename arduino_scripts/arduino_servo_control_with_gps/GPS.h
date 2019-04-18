#ifndef GPS_H
#define GPS_H
#include <Arduino.h>

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"   ///< turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" // turn off antenna status 

#define GPSSerial Serial3

class GPS
{
public:

	// variables
	long baud; // bits per second
	int timeout;     // milliseconds (may need to increase this if data is dropped)
	int num_chars;  // this should be the max number of chars in a NMEA sentence
	String empty_string = "";
	String line = "";   // stores the most recent line of data read from serial
	String gpgga = "";  // holds last GPGGA nmea sentence
	String gprmc = "";  // holds last GPRMC nmea sentence
	bool gpgga_updated = false;
	bool gprmc_updated = false;
	bool updated = false;



	// constructor
	GPS(long baud=9600, int timeout=100)
	{
		this->baud = baud;
		this->timeout = timeout;
		this->num_chars = 512;
		this->line = "";
		this->gpgga = "";
		this->gprmc = "";
		
		// initialize serial communication with the GPS
		GPSSerial.begin(baud);

		// set the timeout for gps serial
	  GPSSerial.setTimeout(timeout); // default, if this line commented out, is 1000 milliseconds
  
		// set gps to only output RMC and GGA lines
	  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	  delay(10);

	  // turn off printing antennae status
	  GPSSerial.println(PGCMD_NOANTENNA);
	  delay(10);

	 	// reserve space for strings
	  this->line.reserve(num_chars); 
	  this->gprmc.reserve(num_chars);
	}

	// get nmea sentences
	bool get_nmea_sentences()
	{
		updated = false; // reset flag
	  int num_nmea_sentences = 0;
	  if(GPSSerial.available() > 0)
	  {
	    line = GPSSerial.readStringUntil('\n');
      if(line.startsWith("$GPGGA"))
      {
      	line.remove(line.length(), 1); // remove the '\r'. Used to be: *XX\r\0, is now: *XX\0
        gpgga = line;
        gpgga_updated = true;
      }
      else if(line.startsWith("$GPRMC"))
      {
      	line.remove(line.length(), 1); // remove the '\r'. Used to be: *XX\r\0, is now: *XX\0
      	gprmc = line;
      	gprmc_updated = true;
      }

      // check if both nmea sentences have been updated
      if(gpgga_updated && gprmc_updated)
      {
      	updated = true; // set flag
      	gpgga_updated = false; // reset flag
      	gprmc_updated = false; // reset flag
      	return true;
      }
	  }
	  else
	  {
	  	return false;
	  }
	}


	// // print all member variables
	// print()
	// {
	// 	Serial.println("print function called...");
	// 	Serial.flush();
	// }








};

#endif



// // function definitions
// int get_nmea_sentences()
// {
//   int num_nmea_sentences = 0;
//   if(GPSSerial.available() > 0)
//   {
//     while(GPSSerial.available() > 0)
//     {
//       inputString = GPSSerial.readStringUntil('\n');
//       if(inputString.charAt(0) != '$')
//       {
//         return -1;
//       }
//       Serial.println(inputString);
//       num_nmea_sentences++;
//       inputString = "";
//     }
//     return num_nmea_sentences;
//   }
//   return 0;
// }

