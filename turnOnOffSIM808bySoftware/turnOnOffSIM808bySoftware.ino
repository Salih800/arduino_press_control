#define POWER_PIN  7 //attached to pin DP9 of the GSM module
#define DELAY_TIME 3000 //3 seconds


void setup()
{
  Serial.begin(9600);
  
  pinMode(POWER_PIN, OUTPUT);
  
  Serial.println("Writing HIGH to pin: " + String(POWER_PIN));
  digitalWrite(POWER_PIN, HIGH);
  Serial.println("Waiting " + String(DELAY_TIME) + " milliseconds...");
  delay (DELAY_TIME);
  Serial.println("Writing LOW to pin: " + String(POWER_PIN));
  digitalWrite (POWER_PIN, LOW);  
  Serial.println("Done!");
}

void loop()
{  

}