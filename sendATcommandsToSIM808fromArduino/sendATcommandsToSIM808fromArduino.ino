#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);  // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop()  // run over and over
{
  while(mySerial.available()){
    Serial.write(mySerial.read());
  }

  while (Serial.available()) {
    mySerial.write(Serial.read());
  }
}