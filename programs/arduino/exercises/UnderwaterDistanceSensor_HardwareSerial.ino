unsigned char buffer_RTT[4] = {0};
uint8_t myCS;
#define COM 0x55
int Distance = 0;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);    // hardware serial on pins 0,1 
}


void loop()
{
  Serial1.write(COM);
  delay(100);
  if(Serial1.available() > 0)
  {
    delay(4);
    if(Serial1.read() == 0xff)
    {    
      buffer_RTT[0] = 0xff;
      for (int i=1; i<4; i++)
      {
        buffer_RTT[i] = Serial1.read();   
      }
      myCS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == myCS)
      {
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        Serial.print("Distance:");
        Serial.print(Distance);
        Serial.println("mm");
      }
    }
  }
}
