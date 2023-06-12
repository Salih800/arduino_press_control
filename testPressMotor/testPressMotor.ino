#define MAGNETIC_LOCK_PIN 6 // Manyetik kilit pin
#define SIM808_POWER_PIN 7  // SIM808 güç pin
#define PRESSUP_PIN 8       // Presi aşağı indirmek için kullanılan pin
#define PRESSDOWN_PIN 9     // Presi yukarı çıkartmak için kullanılan pin


void setup()
{
    Serial.begin(9600);
    pinMode(MAGNETIC_LOCK_PIN, OUTPUT);
    pinMode(SIM808_POWER_PIN, OUTPUT);
    pinMode(PRESSUP_PIN, OUTPUT);
    pinMode(PRESSDOWN_PIN, OUTPUT);
    digitalWrite(MAGNETIC_LOCK_PIN, LOW);
    digitalWrite(SIM808_POWER_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);
    digitalWrite(PRESSDOWN_PIN, LOW);
}

void loop()
{    
    Serial.println("Press down");
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, LOW);
    delay(1000);
    
    Serial.println("Press up");
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, HIGH);
    delay(1000);
    
    Serial.println("Press wait");
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, HIGH);
    delay(1000);
    
    Serial.println("Press stop");
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);
    delay(1000);
}