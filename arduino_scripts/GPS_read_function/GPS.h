#ifndef GPS_H
#define GPS_H

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"   ///< turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" // turn off antenna status 


class GPS
{
public:

	// variables
	HardwareSerial GPSSerial;
	const long baud; // bits per second
	const int timeout;     // milliseconds (may need to increase this if data is dropped)
	const int num_chars;  // this should be the max number of chars in a NMEA sentence
	String line = "";   // stores the most recent line of data read from serial
	String gprmc = "";  // holds last GPRMC nmea sentence




	// constructor
	GPS(HardwareSerial &GPSSerial, const long baud=9600, const int timeout=100)
	{
		this->GPSSerial = GPSSerial;
		this->baud = baud;
		this->timeout = timeout;
		this->num_chars = 256;
		this->line = "";
		this->gprmc = "";

		// initialize serial communication with the GPS
		this->GPSSerial.begin(baud);

		// set the timeout for gps serial
	  this->GPSSerial.setTimeout(timeout); // default, if this line commented out, is 1000 milliseconds
  
		// set gps to only output RMC and GGA lines
	  this->GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);

	  // turn off printing antennae status
	  this->GPSSerial.println(PGCMD_NOANTENNA);

	 	// reserve space for strings
	  this->line.reserve(num_chars); 
	  this->gprmc.reserve(num_chars);


	}








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

