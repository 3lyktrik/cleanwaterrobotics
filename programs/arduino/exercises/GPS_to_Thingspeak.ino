/**************************************************
 *  GPS και αποστολή συντεταγμένων στο ThingSpeak
 *  2026
 **************************************************/

#include "DFRobot_GNSS.h"
#define I2C_COMMUNICATION  // use I2C for communication
DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);

#include <WiFiS3.h>
#include "secrets.h"
#include <ThingSpeak.h> // always include thingspeak header file after other header files and custom macros

char ssid[] = SECRET_SSID;   // WiFi SSID (name) 
char pass[] = SECRET_PASS;   // WiFi password
int status = WL_IDLE_STATUS;
WiFiClient  client;

// η διεύθυνση του thingspeak
char server[] = "api.thingspeak.com";
unsigned long myChannelNumber = SECRET_CH_ID;
const char *myWriteAPIKey = SECRET_WRITE_APIKEY;
const char *myReadAPIKey = SECRET_READ_APIKEY;

// για μέτρηση του χρόνου
unsigned long currentTime;
unsigned long previousTime = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("ΘΕΣΗ GPS");

  while( !gnss.begin() )
  {
    Serial.println("NO GNSS module found...");
    delay(1000);
  }

  gnss.enablePower();   // Enable gnss power 

/** Set GNSS to be used 
 *   eGPS              use gps
 *   eBeiDou           use beidou
 *   eGPS_BeiDou       use gps + beidou
 *   eGLONASS          use glonass
 *   eGPS_GLONASS      use gps + glonass
 *   eBeiDou_GLONASS   use beidou +glonass
 *   eGPS_BeiDou_GLONASS use gps + beidou + glonass
 */
  gnss.setGnss(eGPS_BeiDou_GLONASS);


  // gnss.setRgbOff();
  gnss.setRgbOn();
  // gnss.disablePower();      // Disable GNSS, the data will not be refreshed after disabling  


  // Σύνδεση στο WiFi
  ConnectToWiFi();
  
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop()
{
  // έλεγχος του χρόνου
  currentTime = millis();
  // συγχρονισμός με το ThingSpeak κάθε 30 sec
  if( currentTime - previousTime >= 30000 )
  {
    sTim_t utc = gnss.getUTC();
    sTim_t date = gnss.getDate();
    sLonLat_t lat = gnss.getLat();
    sLonLat_t lon = gnss.getLon();
    double altitude = gnss.getAlt();
    uint8_t satsUsed = gnss.getNumSatUsed();
    double sog = gnss.getSog();               // speed on ground (in knots)
    double cog = gnss.getCog();               // course on ground (in degrees)
  
    Serial.println();
    Serial.print(date.year);
    Serial.print("/");
    Serial.print(date.month);
    Serial.print("/");
    Serial.print(date.date);
    Serial.print("/");
    Serial.print(utc.hour);
    Serial.print(":");
    Serial.print(utc.minute);
    Serial.print(":");
    Serial.print(utc.second);
    Serial.println();
    Serial.println((char)lat.latDirection);
    Serial.println((char)lon.lonDirection);
    
    //Serial.print("lat DDMM.MMMMM = ");
    //Serial.println(lat.latitude, 5);
    //Serial.print("lon DDDMM.MMMMM = ");
    //Serial.println(lon.lonitude, 5);
    Serial.print("lat degree = ");
    Serial.println(lat.latitudeDegree,6);
    Serial.print("lon degree = ");
    Serial.println(lon.lonitudeDegree,6);
  
    Serial.print("Δορυφόροι που χρησιμοποιήθηκαν: ");
    Serial.println(satsUsed);
    Serial.print("Υψόμετρο: ");
    Serial.println(altitude);
    Serial.print("Speed on ground (knots): ");
    Serial.println(sog);
    Serial.print("Course on ground (degrees): ");
    Serial.println(cog);
    Serial.print("GNSS mode: ");
    Serial.println(gnss.getGnssMode());

    ConnectToWiFi();

    ThingSpeak.setField(1, (float)lat.latitudeDegree);
    ThingSpeak.setField(2, (float)lon.lonitudeDegree);
    ThingSpeak.setField(3, (float)sog);
    ThingSpeak.setField(4, (float)cog);
    ThingSpeak.setField(5, (float)satsUsed);
     
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200)      // επιτυχές γράψιμο στο ThingSpeak!  
    {
      Serial.println("Channel writing successful!");
    }
    else
    {
      Serial.println("Problem writing channel... HTTP error code " + String(x));
    }  

    previousTime = currentTime;    
  }
}

void ConnectToWiFi()
{
  // Connect or reconnect to WiFi
  if( WiFi.status() != WL_CONNECTED )
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
}
