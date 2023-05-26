#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10);  // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop()  // run over and over
{
  if (mySerial.available())
    Serial.write(mySerial.read());

  if (Serial.available()) {
    while (Serial.available()) {
      mySerial.write(Serial.read());
    }
    mySerial.println();
  }
}