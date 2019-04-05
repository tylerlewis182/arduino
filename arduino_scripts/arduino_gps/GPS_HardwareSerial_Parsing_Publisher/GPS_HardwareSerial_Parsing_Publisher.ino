// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include "ros.h"
#include <std_msgs/UInt16.h>
#include <sensor_msgs/NavSatFix.h>

ros::NodeHandle nh;



// what's the name of the hardware serial port?
#define GPSSerial Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

/* added by me to try an turn off $PGTOP line from printing: */
#define PGCMD_ANTENNA "$PGCMD,33,1*6C" // request for updates on antenna status 
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 
//  GPS.sendCommand(PGCMD_ANTENNA); // ON
//  GPS.sendCommand(PGCMD_NOANTENNA); // OFF
/* --------------------------------------------------------- */ 



/* added by me to determine type of variables */
void print_type(String var){Serial.println("type is 'String'");}
void print_type(int var){Serial.println("type is 'int'");}
void print_type(float var){Serial.println("type is 'float'");}
void print_type(double var){Serial.println("type is 'double'");}

String s = "stuff";
int i = 0;
float f = 0.0;
double d = 0.0;




/*------------------------------------------- */







/* added by me for ROS                        */
std_msgs::UInt16 gps_msg;
ros::Publisher gps_fix("gps_fix", &gps_msg);

int num = 123;







/*------------------------------------------- */






void setup()
{
  
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA); // ON
  GPS.sendCommand(PGCMD_NOANTENNA); // OFF

  delay(1000);

  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);


  // init ROS nodehandle
  nh.initNode();
  nh.advertise(gps_fix);
  



  
  
}


//
//void loop()
//{
//  while( GPSSerial.available() > 0 )
//  {
//    Serial.write(GPSSerial.read());
//  }
//}










// main
void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();  // if you want to debug, this is a good time to do it!
  if (GPSECHO) // false
    if (c) Serial.print(c);



    
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    Serial.println("---");
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) 
  {
    timer = millis();
  }

  

  

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);

    
    Serial.println("---");
    Serial.print("type of GPS.hour: ");
    print_type(s);
    
    //i = atoi(s.c_str());
    i = s.toInt();
    print_type(i);

    f = s.toFloat();
    print_type(f);

    d = s.toDouble();
    print_type(d);




    gps_msg.data = num;
    gps_fix.publish( &gps_msg );
    nh.spinOnce();

   
    
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//    }
  }

}
