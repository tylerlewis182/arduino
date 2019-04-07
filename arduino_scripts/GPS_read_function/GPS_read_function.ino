


// GPS board is connected to this Arduino Mega Serial3 port (pins 14, 15)
#define GPSSerial Serial3
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"   ///< turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" // turn off antenna status 

// global variables
const long MONITOR_BAUDRATE = 115200; // bits per second
const long GPS_BAUDRATE = 9600; // bits per second
const int TIMEOUT = 100;     // milliseconds (may need to increase this if data is dropped)
const int NUM_CHARS = 256;  // this should be the max number of chars in a NMEA sentence
String inputString = "";    // stores the most recent line of data read from serial

// function prototypes
int print_nmea_string_length();

// setup
void setup()
{
  Serial.begin(MONITOR_BAUDRATE); // bits per second
  GPSSerial.begin(GPS_BAUDRATE);
  // set gps to only output RMC and GGA lines
  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPSSerial.println(PGCMD_NOANTENNA);
  
  GPSSerial.setTimeout(TIMEOUT); // default, if this line commented out, is 1000 milliseconds
  Serial.println("Program started...");
  pinMode(13, OUTPUT);
  inputString.reserve(NUM_CHARS); 
}

int num_nmea_sentences = 0;
// main
void loop()
{
  get_nmea_sentences();
 
 
  
  /* serial monitor outputs the following every 1 second:
   
        $PGTOP,11,2*6E
        $GPGGA,234217.000,3342.7293,N,09635.6730,W,1,07,1.15,197.9,M,-24.7,M,,*55
        $GPGLL,3342.7293,N,09635.6730,W,234217.000,A,A*48
        $GPGSA,A,3,02,17,28,24,12,06,19,,,,,,1.45,1.15,0.88*02
        $GPGSV,3,1,11,06,69,203,20,19,63,348,29,17,53,033,29,28,44,112,19*73
        $GPGSV,3,2,11,02,33,218,17,24,30,296,25,03,15,055,,12,07,311,25*7F
        $GPGSV,3,3,11,22,03,038,,30,02,166,,46,,,27*76
        $GPRMC,234217.000,A,3342.7293,N,09635.6730,W,1.14,171.84,060419,,,A*7A
        $GPVTG,171.84,T,,M,1.14,N,2.11,K,A*30

  */
 
  
}


// function definitions
int get_num_nmea_sentences()
{
  int num_nmea_sentences = 0;
  if(GPSSerial.available() > 0)
  {
    while(GPSSerial.available() > 0)
    {
      inputString = GPSSerial.readStringUntil('\n');
      if(inputString.charAt(0) != '$')
      {
        return -1;
      }
      Serial.println(inputString);
      num_nmea_sentences++;
      inputString = "";
    }
    return num_nmea_sentences;
  }
  return 0;
}

