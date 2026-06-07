/* ******************************************************************************
 * Πρόγραμμα ελέγχου του καταμαράν
 * 
 * 1 - Έλεγχος 2 Κινητήρων DC με πλακέτα L298N (H-Bridge)
 * 2 - Λήψη εντολών ελέγχου από εφαρμογή App Inventor 
 *     μέσω BLE (Bluetooth Low Energy) 
 * 3 - Έλεγχος του σκάφους με κάμερα χειρονομιών
 * 4 - Ενσωμάτωση κάμερας HUSKYLENS
 * 5 - Ενσωμάτωση κινητήρα servo για περιστροφή της HUSKYLENS
 * 6 - Παρακολούθηση πινακίδας με την HUSKYLENS και έλεγχος του σκάφους ανάλογα
 * 7 - Εμφάνιση εικονιδίων στο LED matrix του Arduino R4
 * 8 - Έλεγχος αντλίας καθαρισμού υδάτων με πλακέτα L298N (H-Bridge)
 * 9 - Έλεγχος τροχαλίας και βηματικού κινητήρα για το ανεβοκατέβασμα της αντλίας
 * 10 - Σύνδεση δέκτη GPS και αποστολή συντεταγμένων θέσης στο ThingSpeak μέσω WiFi
 * 
 * 3ο ΓΕΛ Τρικάλων "Οδυσσέας Ελύτης"
 * Ομάδα Ρομποτικής "Clean Water Robotics"
 * Για τον 8ο Διαγωνισμό ΕΛΛΑΚ 2026
 * 
 * ******************************************************************************/

// Εικονίδια για LED matrix
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;
const uint32_t forward[] = {
  0x400e01f,
  0x3f80e00,
  0xe00e00e0
};
const uint32_t backward[] = {
  0xe00e00e,
  0xe03f81,
  0xf00e0040
};
const uint32_t stop[] = {
  0x2642f41f,
  0x830c3fc1,
  0xf8090090
};
const uint32_t happy[] = {
  0x19819,
  0x80000001,
  0x81f8000
};



#include <ArduinoBLE.h>

// Define a custom BLE service and characteristic
BLEService controlService("e9ef1f54-4708-4927-930f-191b2d624cab");  // εδώ δημιουργήσαμε το δικό μας UUID
// στα παρακάτω χαρακτηριστικά, τροποποιήσαμε μόνο τον 8ο χαρακτήρα του UUID, για ευκολία
// Κάθε χαρακτηριστικό αντιστοιχεί σε μια τιμή που θέλουμε να λάβουμε
BLEShortCharacteristic var1Characteristic("e9ef1f55-4708-4927-930f-191b2d624cab", BLEWrite | BLEWriteWithoutResponse | BLERead | BLENotify);
BLEShortCharacteristic var2Characteristic("e9ef1f56-4708-4927-930f-191b2d624cab", BLEWrite | BLEWriteWithoutResponse | BLERead | BLENotify);
BLEUnsignedCharCharacteristic var3Characteristic("e9ef1f57-4708-4927-930f-191b2d624cab", BLEWrite | BLEWriteWithoutResponse);
BLEUnsignedCharCharacteristic var4Characteristic("e9ef1f58-4708-4927-930f-191b2d624cab", BLEWrite | BLEWriteWithoutResponse);
BLEUnsignedCharCharacteristic var5Characteristic("e9ef1f59-4708-4927-930f-191b2d624cab", BLEWrite | BLEWriteWithoutResponse);

// pins ελέγχου του μοτέρ 1
const int enA = 10;
const int in1 = 9;
const int in2 = 8;
// pins ελέγχου του μοτέρ 2
const int enB = 11;
const int in3 = 13;
const int in4 = 12;

// Define pin connections & stepper motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
//const int stepsPerRevolution = 200; // για το συγκεκριμένο μοτέρ
const int fullLength = 800;   // 800 = 4 * 200 βήματα, άρα 4 πλήρεις περιστροφές
int strofes=0;        // για να μετράμε τα βήματα του stepper motor

// Η κλίμακα ταχύτητας των μοτέρ είναι από -4 έως 4, όπως φαίνεται στον πίνακα:
/*****************************
 * Κλίμακα  ->  Ταχύτητα (PWM)
 *      4   ->  μπροστά 255
 *      3   ->  μπροστά 230
 *      2   ->  μπροστά 205
 *      1   ->  μπροστά 180
 *      0   ->  0
 *     -1   ->  πίσω 180
 *     -2   ->  πίσω 205
 *     -3   ->  πίσω 230
 *     -4   ->  πίσω 255
 *****************************/

short leftSpeed = 0;
short rightSpeed = 0;

// για εντολή ενεργοποίησης του ελέγχου του σκάφους με χειρονομίες
bool enableGestures = false;
// για εντολή ενεργοποίησης της παρακολούθησης της πινακίδας
bool followTag = false;
// για εντολή καθαρισμού του νερού
bool cleanWater = false;
bool cleaning=false;    // η κατάσταση καθαρισμού

// για εντοπισμό της πινακίδας
bool found=false;

#include "DFRobot_GestureFaceDetection.h"
// Define the device ID for the GestureFaceDetection sensor
#define DEVICE_ID 0x72
// Create an instance of DFRobot_GestureFaceDetection_I2C with the specified device ID
DFRobot_GestureFaceDetection_I2C gfd(DEVICE_ID);
// Buffer for formatted output
char str[100];



#include <Servo.h>
Servo myservo;  // create servo object to control the servo
const int servoPin = 6;
int pos = 0;    // variable to store the servo position

#include "HUSKYLENS.h"
HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
int ID0 = 0; //not learned results. Grey result on HUSKYLENS screen
int ID1 = 1; //first learned results. colored result on HUSKYLENS screen


// για τον υποβρύχιο αισθητήρα απόστασης
unsigned char buffer_RTT[4] = {0};
uint8_t myCS;
#define COM 0x55
int distance = 0;



// GPS και αποστολή δεδομένων στο ThingSpeak μέσω WiFi
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
  matrix.begin();     // αρχικοποίηση LED matrix του Arduino R4

  // ορισμός pin
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(servoPin, OUTPUT);

  Serial.begin(115200);     // USB serial
  Serial1.begin(115200);    // hardware serial on pins 0,1, για τον υποβρύχιο αισθητήρα απόστασης

  // Initialize BLE
  if( !BLE.begin() )
  {
    Serial.println("Failed to start BLE.");
    Serial.println("Press Reset...");
    while(1)
      ;
  }
  else
  {
    Serial.println("Bluetooth ok!");
  }

  // Set the device name and advertise the service
  BLE.setLocalName("CatamaranPeripheral");
  BLE.setAdvertisedService(controlService);

  // Attach the characteristics to the service
  controlService.addCharacteristic(var1Characteristic);
  controlService.addCharacteristic(var2Characteristic);
  controlService.addCharacteristic(var3Characteristic);
  controlService.addCharacteristic(var4Characteristic);
  controlService.addCharacteristic(var5Characteristic);

  // Register the service with the BLE stack
  BLE.addService(controlService);

  // Set initial values for the characteristics
  var1Characteristic.writeValue(0);   // ταχύτητα του μοτέρ 1
  var2Characteristic.writeValue(0);   // ταχύτητα του μοτέρ 2
  var3Characteristic.writeValue(0);   // για ενεργοποίηση χειρονομιών (enableGestures)
  var4Characteristic.writeValue(0);   // για ενεργοποίηση παρακολούθησης πινακίδας (followTag)
  var5Characteristic.writeValue(0);   // για ενεργοποίηση καθαρισμού του νερού (cleanWater)

  // Start advertising the BLE service
  BLE.advertise();
  Serial.println("BLE Catamaran Peripheral is now advertising...");

  // Wait for the gesture sensor to start.
  delay(3000);
  // Initialize I2C communication
  gfd.begin(&Wire);
  while( !gfd.begin() )
  {
    Serial.println("Communication with Gesture Camera failed, please check connection...");
    delay(1000);
  }

  // κάμερα HUSKYLENS
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
      Serial.println(F("Huskylens initialization failed..."));
      Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
      Serial.println(F("2.Please recheck the connection."));
      delay(100);
  }
  // ενεργοποίηση του αλγόριθμου αναγνώρισης πινακίδων (April Tags) του HUSKYLENS
  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  // servo
  myservo.attach(servoPin);  // attaches the servo on pin 6 to the servo object
  pos = 90;           // αρχική τοποθέτηση του servo στη μέση
  myservo.write(pos);
  delay(500);

  // GPS
  while( !gnss.begin() )
  {
    Serial.println("NO GNSS module found...");
    delay(1000);
  }
  gnss.enablePower();   // Enable gnss power 
  gnss.setGnss(eGPS_BeiDou_GLONASS);  // use gps + beidou + glonass
  gnss.setRgbOn();
  // gnss.disablePower();      // Disable GNSS, the data will not be refreshed after disabling  

  // Σύνδεση στο WiFi
  ConnectToWiFi();
  
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}


void loop()
{
  // Wait for a central device to connect
  BLEDevice central = BLE.central();

  if( central )
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // Stay in loop while the central is connected
    while( central.connected() )
    {
      // έλεγχος του χρόνου για αποστολή στο ThingSpeak
      currentTime = millis();
      // συγχρονισμός με το ThingSpeak κάθε 60 sec
      if( currentTime - previousTime >= 60000 )
      {
        sendToThingSpeak();
        previousTime = currentTime;
      }

      // Check if the characteristics has been updated by the central
      if( var1Characteristic.written() )
      {
        // Read the new value from the characteristic
        short var1Value = var1Characteristic.value();
        UpdateSpeedMotor1(var1Value);
      }
      if( var2Characteristic.written() )
      {
        // Read the new value from the characteristic
        short var2Value = var2Characteristic.value();
        UpdateSpeedMotor2(var2Value);
      }
      if( var3Characteristic.written() )
      {
        // Read the new value from the characteristic
        unsigned char var3Value = var3Characteristic.value();
        if(var3Value==1)
        {
          enableGestures=true;
        }
        else
        {
          enableGestures=false;
        }
      }
      if( var4Characteristic.written() )
      {
        // Read the new value from the characteristic
        unsigned char var4Value = var4Characteristic.value();
        if(var4Value==1)
        {
          followTag=true;
        }
        else
        {
          followTag=false;
        }
      }
      if( var5Characteristic.written() )
      {
        // Read the new value from the characteristic
        unsigned char var5Value = var5Characteristic.value();
        if(var5Value==1)
        {
          cleanWater=true;
        }
        else
        {
          cleanWater=false;
        }
      }      


      // Έλεγχος για χειρονομίες
      if(enableGestures)
      {
        // Check if any faces are detected
        if( gfd.getFaceNumber() > 0 )
        {
          // Retrieve face score and location
          uint16_t faceScore = gfd.getFaceScore();
          uint16_t faceX = gfd.getFaceLocationX();
          uint16_t faceY = gfd.getFaceLocationY();

          // Print the face detection results
          sprintf(str, "detect face at (x = %d, y = %d, score = %d)\n", faceX, faceY, faceScore);
          Serial.print(str);

          // Print the gesture detection results
          // - 1: LIKE (👍) - blue
          // - 2: OK (👌) - green
          // - 3: STOP (🤚) - red
          // - 4: YES (✌) - yellow
          // - 5: SIX (🤙) - purple
          uint16_t gestureType = gfd.getGestureType();
          uint16_t gestureScore = gfd.getGestureScore();

          // Print the gesture detection results
          sprintf(str, "detect gesture %d, score = %d\n", gestureType, gestureScore);
          Serial.print(str);
          if( gestureType==1 )        // 👍: μπροστά
          {
            if(leftSpeed!=4)
            {
              UpdateSpeedMotor1(4);
              var1Characteristic.writeValue(4);
            }
            if(rightSpeed!=4)
            {
              UpdateSpeedMotor2(4);
              var2Characteristic.writeValue(4);
            }
            matrix.loadFrame(forward); // εμφάνισε εικονίδιο βέλος μπροστά
          }
          else if( gestureType==3 )   // 🤚: stop
          {
            if(leftSpeed!=0)
            {
              UpdateSpeedMotor1(0);
              var1Characteristic.writeValue(0);
            }
            if(rightSpeed!=0)
            {
              UpdateSpeedMotor2(0);
              var2Characteristic.writeValue(0);
            }
            matrix.loadFrame(stop); // εμφάνισε εικονίδιο stop

          }
          else if( gestureType==5 )     // 🤙: πίσω
          {
            if(leftSpeed!=-4)
            {
              UpdateSpeedMotor1(-4);
              var1Characteristic.writeValue(-4);
            }
            if(rightSpeed!=-4)
            {
              UpdateSpeedMotor2(-4);
              var2Characteristic.writeValue(-4);
            }
            matrix.loadFrame(backward); // εμφάνισε εικονίδιο βέλος πίσω
          } 
        }
      }
      else
      {
        matrix.loadFrame(happy); // εμφάνισε εικονίδιο happy
      }

      // Έλεγχος για εντολή παρακολούθησης πινακίδας
      if( followTag )
      {
        if( !found )  // αν δεν έχει βρεθεί η πινακίδα
        {
            // τότε το σκάφος σταματά
            if(leftSpeed!=0)
            {
              UpdateSpeedMotor1(0);
              var1Characteristic.writeValue(0);
            }
            if(rightSpeed!=0)
            {
              UpdateSpeedMotor2(0);
              var2Characteristic.writeValue(0);
            }
          // η κάμερα ξεκινά από τέρμα αριστερά και σαρώνει το χώρο μέχρι τέρμα δεξιά,
          // μέχρι να εντοπίσει την πινακίδα
          pos=180;
          while( pos>0 && !found )
          {
            myservo.write(pos);
            delay(15);
            if (huskylens.requestBlocks(ID1))
            {
              if ( huskylens.count(ID1)>0 )   // αν βρέθηκε μια πινακίδα
              {
                found=true;
              }
              else
              {
                pos--;
              }
            }
          }
        }
        if( found )     // αν εντοπίστηκε η πινακίδα, τότε 
        {
          // μέχρι να κεντραριστεί η κάμερα,
          // θα παρακολουθεί συνεχώς την πινακίδα με τη βοήθεια του servo
          //request blocks tagged ID == ID1 from HUSKYLENS
          if (huskylens.requestBlocks(ID1))
          {
            if ( huskylens.count(ID1)>0 )   // αν βρέθηκε μια πινακίδα
            {
              HUSKYLENSResult result = huskylens.get(ID1, 0);
              int x = result.xCenter;
  
              // αντί για τη μέση τιμή (160) χρησιμοποιούμε τις τιμές 135
              // και 185, για να δώσουμε μια ανοχή και να αποφύγουμε τις
              // συνεχείς κινήσεις του servo μοτέρ
              if( x<135 )
              {
                // το servo κινείται προς τα αριστερά (μέχρι 180 μοίρες)
                if( pos<180 ) pos++;
                else pos=180;
              }
              else if( x>185 )
              {
                // το servo κινείται προς τα δεξιά (μέχρι 0 μοίρες)
                if( pos>0 ) pos--;
                else pos=0;
              }
              myservo.write(pos);
              delay(15);
            }
          }

          // αφού η κάμερα έστριψε προς την πινακίδα, κατόπιν θα αρχίσει να στρίβει όλο το σκάφος
          if( pos>100 )  // δηλ. αν η κάμερα έστριψε προς τα αριστερά
          {
            // τότε και το σκάφος στρίβει προς τα αριστερά
            if(leftSpeed!=-3) 
            {
              UpdateSpeedMotor1(-3);
              var1Characteristic.writeValue(-3);
            }
            if(rightSpeed!=3)
            {
              UpdateSpeedMotor2(3);
              var2Characteristic.writeValue(3);
            }
          }
          else if( pos<80 )  // αλλιώς αν η κάμερα έστριψε προς τα δεξιά
          {
            // τότε και το σκάφος στρίβει προς τα δεξιά
            if(leftSpeed!=3)
            {
              UpdateSpeedMotor1(3);
              var1Characteristic.writeValue(3);
            }
            if(rightSpeed!=-3)
            {
              UpdateSpeedMotor2(-3);
              var2Characteristic.writeValue(-3);
            }
          }
          else   // αλλιώς, δηλ. αν η κάμερα κεντραρίστηκε
          {
            // τότε το σκάφος σταματά
            if(leftSpeed!=0)
            {
              UpdateSpeedMotor1(0);
              var1Characteristic.writeValue(0);
            }
            if(rightSpeed!=0)
            {
              UpdateSpeedMotor2(0);
              var2Characteristic.writeValue(0);
            }
          }
        }
        else    // αν δεν εντοπίστηκε η πινακίδα, τότε στρίψε λίγο το σκάφος για να ξανασαρώσει παραπέρα
        {
          // το σκάφος στρίβει για λίγο (2 sec) προς τα δεξιά
          UpdateSpeedMotor1(1);
          var1Characteristic.writeValue(1);
          UpdateSpeedMotor2(-1);
          var2Characteristic.writeValue(-1);
          delay(2000);
          UpdateSpeedMotor1(0);
          var1Characteristic.writeValue(0);
          UpdateSpeedMotor2(0);
          var2Characteristic.writeValue(0);
        }
      }
      else  // δηλ. αν δεν είναι τσεκαρισμένο το followTag
      {
        found=false;
      }

      // Έλεγχος για εντολή καθαρισμού του νερού
      if(cleanWater && !cleaning)
      {
        // Αρχικά ξετυλίγουμε την τροχαλία
        digitalWrite(dirPin, HIGH);
        strofes=0;      // μετράμε τις στροφές που θα κάνει η τροχαλία μέχρι το βυθό
        distance=getUnderWaterDistance();
        // ξετυλίγουμε μέχρι 10cm από το βυθό, αγνοώντας όταν είναι έξω από το νερό (distance=0), και το πολύ έως 4 πλήρεις περιστροφές του stepper
        while( (distance>100 || distance==0) && strofes<=800 )
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(3000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(3000);
          strofes++;
          if( strofes%40 == 0 )     // κάθε 40 βήματα του stepper, ξαναμετράμε το βάθος
            distance=getUnderWaterDistance();
        }
        cleaning=true;
      }
      // Έλεγχος για εντολή τερματισμού του καθαρισμού νερού
      if(!cleanWater && cleaning)
      {
        // Τυλίγουμε την τροχαλία
        digitalWrite(dirPin, LOW);
        // περιστρέφουμε το stepper ανάποδα, για όσες στροφές ξετυλίχτηκε προηγουμένως
        for (int x = 0; x < strofes; x++)
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(3000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(3000);
        }
        strofes=0;
        cleaning=false;        
      }
    }

    // Handle disconnection
    Serial.print("Disconnected from: ");
    Serial.println(central.address());
  }
}

// Αριστερός κινητήρας
void UpdateSpeedMotor1(int newSpeed)
{
  if( leftSpeed == newSpeed )
    return;

  int arxiki;
  int teliki;
  if(leftSpeed==0) arxiki=0;
  else arxiki = 180 + (abs(leftSpeed)-1)*25;  // κάθε βαθμίδα της κλίμακας ταχύτητας ανεβαίνει 25 κατά PWM,
  if(newSpeed==0) teliki=0;
  else teliki = 180 + (abs(newSpeed)-1)*25;   // δηλαδή 1->180, 2->205, 3->230, 4->255
  
  if( leftSpeed >= 0 && newSpeed >= 0 )  // μπροστά
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    if( teliki > arxiki )    // σταδιακή αύξηση ταχύτητας
    {
      GradualIncreaseSpeed(enA, arxiki, teliki);
    }
    else      // μείωση κατευθείαν
    {
      analogWrite(enA, teliki);
    }
  }
  else if( leftSpeed <= 0 && newSpeed <= 0 )   // πίσω
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    if( teliki > arxiki )    // σταδιακή αύξηση ταχύτητας
    {
      GradualIncreaseSpeed(enA, arxiki, teliki);      
    }
    else      // μείωση κατευθείαν
    {
      analogWrite(enA, teliki);
    }
  }
  else if( leftSpeed > 0 && newSpeed < 0 )   // αλλαγή από μπροστά πίσω
  {
    // αρχικά μηδενισμός ταχύτητας
    analogWrite(enA, 0);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    GradualIncreaseSpeed(enA, 0, teliki);
  }
  else if( leftSpeed < 0 && newSpeed > 0 )   // αλλαγή από πίσω μπροστά
  {
    // αρχικά μηδενισμός ταχύτητας
    analogWrite(enA, 0);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    GradualIncreaseSpeed(enA, 0, teliki);
  }  
  leftSpeed = newSpeed;
}

// Δεξιός κινητήρας
void UpdateSpeedMotor2(int newSpeed)
{
  if( rightSpeed == newSpeed )
    return;

  int arxiki;
  int teliki;
  if(rightSpeed==0) arxiki=0;
  else arxiki = 180 + (abs(rightSpeed)-1)*25;  // κάθε βαθμίδα της κλίμακας ταχύτητας ανεβαίνει 25 κατά PWM,
  if(newSpeed==0) teliki=0;
  else teliki = 180 + (abs(newSpeed)-1)*25;   // δηλαδή 1->180, 2->205, 3->230, 4->255
  
  if( rightSpeed >= 0 && newSpeed >= 0)  // μπροστά
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    if( teliki > arxiki )    // σταδιακή αύξηση ταχύτητας
    {
      GradualIncreaseSpeed(enB, arxiki, teliki);
    }
    else      // μείωση κατευθείαν
    {
      analogWrite(enB, teliki);
    }
  }
  else if( rightSpeed <= 0 && newSpeed <= 0 )   // πίσω
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    if( teliki > arxiki )    // σταδιακή αύξηση ταχύτητας
    {
      GradualIncreaseSpeed(enB, arxiki, teliki);      
    }
    else      // μείωση κατευθείαν
    {
      analogWrite(enB, teliki);
    }
  }
  else if( rightSpeed > 0 && newSpeed < 0 )   // αλλαγή από μπροστά πίσω
  {
    // αρχικά μηδενισμός ταχύτητας
    analogWrite(enB, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    GradualIncreaseSpeed(enB, 0, teliki);
  }
  else if( rightSpeed < 0 && newSpeed > 0 )   // αλλαγή από πίσω μπροστά
  {
    // αρχικά μηδενισμός ταχύτητας
    analogWrite(enB, 0);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    GradualIncreaseSpeed(enB, 0, teliki);
  }  
  rightSpeed = newSpeed;
}

// Για σταδιακή μεταβολή της ταχύτητας
void GradualIncreaseSpeed(int motorEnablePin, int arxiki, int teliki)
{
  for( int i = arxiki; i <= teliki; i+=5 )
  {
    analogWrite(motorEnablePin, i);
    //Serial.println(i);
    delay(10);
  }
}


// μέτρηση του βάθος με τον υποβρύχιο αισθητήρα απόστασης
// η παρακάτω συνάρτηση έχει βελτιωθεί από το chatGPT, για να λειτουργούν σωστά οι χρονισμοί μεταξύ BLE, UART κλπ
int getUnderWaterDistance()
{
  int Distance = 0;

  // καθάρισε παλιά bytes
  while (Serial1.available())
  {
    Serial1.read();
  }

  // send command
  Serial1.write(COM);

  unsigned long startTime = millis();

  // περίμενε μέχρι να έρθουν 4 bytes
  while (Serial1.available() < 4)
  {
    BLE.poll();   // ΠΟΛΥ σημαντικό στο UNO R4 WiFi

    if (millis() - startTime > 30)
    {
      Serial.println("Sensor timeout");
      return 0;
    }
  }

  // διάβασε packet
  for (int i = 0; i < 4; i++)
  {
    buffer_RTT[i] = Serial1.read();
  }

  // έλεγχος header
  if (buffer_RTT[0] != 0xFF)
  {
    Serial.println("Header error");
    return 0;
  }

  // checksum
  myCS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];

  if (buffer_RTT[3] != myCS)
  {
    Serial.println("Checksum error");
    return 0;
  }

  Distance = (buffer_RTT[1] << 8) | buffer_RTT[2];

  Serial.print("Distance: ");
  Serial.print(Distance);
  Serial.println(" mm");

  return Distance;
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


void sendToThingSpeak()
{
  sTim_t utc = gnss.getUTC();
  sTim_t date = gnss.getDate();
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  double altitude = gnss.getAlt();
  uint8_t satsUsed = gnss.getNumSatUsed();
  double sog = gnss.getSog();               // speed on ground (in knots)
  double cog = gnss.getCog();               // course on ground (in degrees)

/* Μηνύματα στο Serial Monitor
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
*/
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
}
