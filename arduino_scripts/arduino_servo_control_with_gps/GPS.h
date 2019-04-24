#ifndef GPS_H
#define GPS_H


// #define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"   ///< turn on GPRMC and GGA
// #define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
// #define PGCMD_NOANTENNA "$PGCMD,33,0*6D" // turn off antenna status 

/** 


  @file Adafruit_GPS.h

  This is the Adafruit GPS library - the ultimate GPS library
  for the ultimate GPS module!

  Tested and works great with the Adafruit Ultimate GPS module
  using MTK33x9 chipset
      ------> http://www.adafruit.com/products/746
  Pick one up today at the Adafruit electronics shop
  and help support open source hardware & software! -ada

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above must be included in any redistribution


 Different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
 Note that these only control the rate at which the position is echoed, to actually speed up the
 position fix you must also send one of the position fix rate commands below too. */
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F"  ///< Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"   ///< Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"              ///<  1 Hz
#define PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B"               ///<  2 Hz
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"               ///<  5 Hz
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"               ///< 10 Hz
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C"  ///< Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"   ///< Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"              ///< 1 Hz
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"               ///< 5 Hz
// Can't fix position faster than 5 times a second!

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C" ///< 57600 bps
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"   ///<  9600 bps

#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"   ///< turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"      ///< turn off output

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"          ///< Start logging data
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"            ///< Stop logging data
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"   ///< Acknowledge the start or stop command
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"         ///< Query the logging status
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"        ///< Erase the log flash data
#define LOCUS_OVERLAP 0                               ///< If flash is full, log will overwrite old data with new logs
#define LOCUS_FULLSTOP 1                              ///< If flash is full, logging will stop

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"              ///< Enable search for SBAS satellite (only works with 1Hz output rate)
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"              ///< Use WAAS for DGPS correction data

#define PMTK_STANDBY "$PMTK161,0*28"              ///< standby command & boot successful message
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  ///< Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"              ///< Wake up

#define PMTK_Q_RELEASE "$PMTK605*31"              ///< ask for the release and version


#define PGCMD_ANTENNA "$PGCMD,33,1*6C"            ///< request for updates on antenna status
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"          ///< don't show antenna status messages

#define MAXWAITSENTENCE 10   ///< how long to wait when we're looking for a response


#define GPSSerial Serial3


// GPS
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
    GPSSerial.begin(this->baud);

    // set the timeout for gps serial
    GPSSerial.setTimeout(this->timeout); // default, if this line commented out, is 1000 milliseconds
  
    // set gps to only output RMC and GGA lines
    GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPSSerial.flush();
    
    // turn off printing antennae status
    GPSSerial.println(PGCMD_NOANTENNA);
    GPSSerial.flush();

    // set gps update rate to 5 Hz (10Hz does not work...)
    GPSSerial.println(PMTK_SET_NMEA_UPDATE_5HZ);
    GPSSerial.flush();

    // reserve space for strings
    this->line.reserve(this->num_chars); 
    this->gprmc.reserve(this->num_chars);
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
        line.trim(); // remove the '\r'. Used to be: *XX\r\0, is now: *XX\0
        gpgga = line;
        gpgga_updated = true;
      }
      else if(line.startsWith("$GPRMC"))
      {
        line.trim(); // remove the '\r'. Used to be: *XX\r\0, is now: *XX\0
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
  //  Serial.println("print function called...");
  //  Serial.flush();
  // }

};

#endif


