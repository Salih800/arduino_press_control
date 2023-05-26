static const int PowerPin =9; //attached to pin DP9 of the GSM module

void setup()
{
  pinMode(PowerPin, OUTPUT);
  //to turn on the GSM module -software switch instead Power Key
  digitalWrite(PowerPin, HIGH);
  delay (1000);
  digitalWrite (PowerPin, LOW);  
  
  delay (10000); //delay set for 10 seconds
  
  //to turn off the GSM module
  digitalWrite(PowerPin, HIGH);
  delay (3000);
  digitalWrite (PowerPin, LOW);

}

void loop()
{  

}