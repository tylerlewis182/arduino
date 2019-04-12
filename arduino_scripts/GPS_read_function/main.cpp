#include "Adafruit_GPS.h"


// GPS board is connected to this Arduino Mega Serial3 port (pins 14, 15)
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// global variables
const long MONITOR_BAUDRATE = 115200; // bits per second

// timer for debugging
uint32_t timer = millis();
uint32_t counter = 0;

// setup
void setup()
{
  Serial.begin(MONITOR_BAUDRATE); // bits per second
  Serial.println("Program started...");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // only send RMC and GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);

  delay(2000);
}





// main
void loop()
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  counter++; // count how many times the read() method is called

  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }  

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Counter = ");
    Serial.println(counter);
    counter = 0; // reset the counter
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) 
    {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}





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
 