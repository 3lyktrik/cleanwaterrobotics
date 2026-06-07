#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

#include "HUSKYLENS.h"
HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
int ID0 = 0; //not learned results. Grey result on HUSKYLENS screen
int ID1 = 1; //first learned results. colored result on HUSKYLENS screen

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    // the algorithm on HUSKYLENS
    huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

    myservo.attach(6);  // attaches the servo on pin 6 to the servo object
    pos = 90;
    myservo.write(pos);  // αρχική τοποθέτηση του servo στη μέση
    delay(500);
}

void loop()
{
    //request blocks tagged ID == ID1 from HUSKYLENS
    if (huskylens.requestBlocks(ID1))
    {
      if ( huskylens.count(ID1)>0 )   // αν βρέθηκε μια πινακίδα
      {
        HUSKYLENSResult result = huskylens.get(ID1, 0);
        int x = result.xCenter;
        // αντί για τη μέση τιμή (160) χρησιμοποιούμε τις τιμές 135
        // και 185, για να δώσουμε μια ανοχή και να αποφύγουμε τις
        // συνεχείς κινήσεις του μοτέρ
        if( x<135 )
        {
          // κινήσου προς τα αριστερά (μέχρι 180 μοίρες)
          if( pos<180 ) pos++;
          else pos=180;
        }
        else if( x>185 )
        {
          // κινήσου προς τα δεξιά (μέχρι 0 μοίρες)
          if( pos>0 ) pos--;
          else pos=0;
        }
        myservo.write(pos);
        delay(15);
      }
    }
}
