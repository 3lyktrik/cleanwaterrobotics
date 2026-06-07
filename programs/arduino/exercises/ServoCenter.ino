#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position


void setup()
{
    myservo.attach(6);  // attaches the servo on pin 6 to the servo object
    pos = 90;
    myservo.write(pos);  // αρχική τοποθέτηση του servo στη μέση
}

void loop()
{
  myservo.write(0);
  delay(2000);
  myservo.write(180);
  delay(2000);
  myservo.write(90);
  delay(2000);
}
