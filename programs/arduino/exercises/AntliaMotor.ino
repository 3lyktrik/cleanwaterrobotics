// pins ελέγχου του μοτέρ της αντλίας
const int enC = 5;
const int in5 = 4;
const int in6 = 7;

void setup()
{
  pinMode(enC, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
}

void loop()
{
    analogWrite(enC, 255);
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
}
