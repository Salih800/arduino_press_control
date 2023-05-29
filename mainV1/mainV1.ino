// #include <SoftwareSerial.h> // SIM808 kütüphanesi
#include <U8g2lib.h> // Ekran kütüphanesi
#include <Adafruit_VL53L0X.h> // Mesafe sensörü kütüphanesi
#include <INA226.h> // Akım ve voltaj sensörü kütüphanesi
#include <math.h> //loads the more advanced math functions

// Pinler
#define TEMPERATURE_PIN A0 // Sıcaklık sensörü 
#define COVER_PIN 4 // Kapak sensörü pin
#define DOOR_PIN 5 // Kapı sensörü pin
#define MAGNETIC_LOCK_PIN 6 // Manyetik kilit pin
#define PRESSUP_PIN 8 // Presi aşağı indirmek için kullanılan pin
#define PRESSDOWN_PIN 9 // Presi yukarı çıkartmak için kullanılan pin
#define SIM808_RX_PIN 10 // SIM808 RX pin
#define SIM808_TX_PIN 11 // SIM808 TX pin

// Sabitler
#define SCREEN_WIDTH 128 // Ekran genişliği
#define SCREEN_HEIGHT 64 // Ekran yüksekliği

const long PRESS_TIME = 27000; // Pres süresi
const long PRESS_DOWN_TIME = PRESS_TIME;
const long PRESS_WAIT_TIME = PRESS_TIME + 1000;
const long PRESS_UP_TIME = (PRESS_TIME * 2) + 1000;

// Nesneler
Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();  // Mesafe sensörü
INA226 powerSensor; // Akım ve voltaj sensörü
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ

// Değişkenler
int doorState = 0; // Kapı durumu
int coverState = 0; // Kapak durumu
int magneticLockState = 0; // Manyetik kilit durumu
int occupancy = 0; // Doluluk oranı
int distance = 0; // Mesafe değişkeni
float voltage = 0.0; // Voltaj değişkeni
float current = 0.0; // Akım değişkeni
float power = 0.0; // Güç değişkeni
float temperature = 0.0; // Sıcaklık değişkeni
float temperature2 = 0.0; // Sıcaklık değişkeni 2

bool isPressing = false; // Pres durumu

// Zaman değişkenleri
unsigned long pressTimer = 0; // Pres zamanlayıcısı

// Fonksiyonlar
double Thermister(int RawADC); // Sıcaklık fonksiyonu

void setup()
{
    // Ekran başlatılıyor
    String startingText = "BASLATILIYOR...";
    u8g2.begin(); 
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    int w = u8g2.getUTF8Width(startingText.c_str());
    u8g2.drawStr((SCREEN_WIDTH/2) - (w/2), 32, startingText.c_str());
    u8g2.sendBuffer();

    // Pin modları ayarlanıyor
    pinMode(COVER_PIN, INPUT);
    pinMode(DOOR_PIN, INPUT);
    pinMode(MAGNETIC_LOCK_PIN, OUTPUT);
    pinMode(PRESSDOWN_PIN, OUTPUT);
    pinMode(PRESSUP_PIN, OUTPUT);

    // Manyetik kilit varsayılan olarak açık
    digitalWrite(MAGNETIC_LOCK_PIN, HIGH);
    
    // Presler varsayılan olarak kapalı
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);

    // Pin değerleri okunuyor
    doorState = digitalRead(DOOR_PIN);
    coverState = digitalRead(COVER_PIN);
    magneticLockState = digitalRead(MAGNETIC_LOCK_PIN);

    // Serial port başlatılıyor
    Serial.begin(9600);
    Serial.println("Starting...");
    
    // Mesafe sensörü başlatılıyor
    distanceSensor.begin();

    // Akım ve voltaj sensörü başlatılıyor
    // INA.begin();
    // INA.setMaxCurrentShunt(0.1, 0.005); // 0.1A, 0.005 ohm

    // INA.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    powerSensor.begin();
    powerSensor.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    powerSensor.calibrate(0.01, 9);
}

void loop()
{
    Serial.print("Running: ");
    Serial.println(millis());
    
    distance = readDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    occupancy = checkOccupancy();
    Serial.print("Occupancy: ");
    Serial.println(occupancy);

    voltage = readVoltage();
    Serial.print("Voltage: ");
    Serial.println(voltage);

    current = readCurrent();
    Serial.print("Current: ");
    Serial.println(current);

    power = readPower();
    Serial.print("Power: ");
    Serial.println(power);

    temperature = readTemperature();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    temperature2 = Thermister(analogRead(TEMPERATURE_PIN));
    Serial.print("Temperature2: ");
    Serial.println(temperature2);

    // Pres zamanlayıcısı
    if (!isPressing && occupancy >= 70 && checkPressReady()) {
        Serial.println("Starting press...");
        pressTimer = millis();
        isPressing = true;
        digitalWrite(MAGNETIC_LOCK_PIN, LOW);
        startPress();
    }
    else if (isPressing && checkPressReady()) {
        Serial.println("Continuing press...");
        startPress();
    }
    else {
        stopPress();
    }

    // Ekran güncelleniyor
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_u8glib_4_tr); // u8g2_font_ncenB08_tr

        u8g2.setCursor(0, 10);
        u8g2.print("Mesafe: ");
        u8g2.print(distance);
        u8g2.println(" mm");

        u8g2.setCursor(0, 20);
        u8g2.print("Voltaj: ");
        u8g2.print(voltage);
        u8g2.println(" V");

        u8g2.setCursor(0, 30);
        u8g2.print("Akim: ");
        u8g2.print(current);
        u8g2.println(" A");

        u8g2.setCursor(0, 40);
        u8g2.print("Guc: ");
        u8g2.print(power);
        u8g2.println(" W");

        u8g2.setCursor(0, 50);
        u8g2.print("Sicaklik: ");
        u8g2.print(temperature);
        u8g2.println(" C");

        u8g2.setCursor(0, 60);
        u8g2.print("Sicaklik2: ");
        u8g2.print(temperature2);
        u8g2.println(" C");

        u8g2.setCursor(60, 10);
        u8g2.print("Doluluk: ");
        u8g2.print(checkOccupancy());
        u8g2.println(" %");

        u8g2.setCursor(60, 20);
        u8g2.print("Kapak: ");
        u8g2.println(checkCoverState());

        u8g2.setCursor(60, 30);
        u8g2.print("Kapi: ");
        u8g2.println(checkDoorState());

        u8g2.setCursor(60, 40);
        u8g2.print("M.Kilit: ");
        u8g2.println(checkMagneticLockState());

        u8g2.setCursor(60, 50);
        u8g2.print("Pres: ");
        u8g2.println(checkPressState());

    } while (u8g2.nextPage());
    
}

// Pres yapmaya hazır mı kontrol eden fonksiyon
bool checkPressReady() {
    if (doorState == 1 && coverState == 0) {
        return true;
    }
    else {
        return false;
    }
}

// Pres durumunu kontrol eden fonksiyon
String checkPressState() {
    int pressDownState = digitalRead(PRESSDOWN_PIN);
    int pressUpState = digitalRead(PRESSUP_PIN);
    if (pressDownState == 1 && pressUpState == 0) {
        return "Asagi";
    }
    else if (pressDownState == 0 && pressUpState == 1) {
        return "Yukari";
    }
    else if (pressDownState == 1 && pressUpState == 1) {
        return "Bekleme";
    }
    else {
        return "Kapali";
    }
}

// Pres durdurma fonksiyonu
void stopPress() {
    Serial.println("Stopping press...");
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);
    digitalWrite(MAGNETIC_LOCK_PIN, HIGH);
    isPressing = false;
}

// Pres yapma fonksiyonu
void startPress() {
    if (millis() - pressTimer <= PRESS_DOWN_TIME) {
        digitalWrite(PRESSDOWN_PIN, HIGH);
        digitalWrite(PRESSUP_PIN, LOW);
    }
    else if (millis() - pressTimer > PRESS_DOWN_TIME && millis() - pressTimer <= PRESS_WAIT_TIME) {
        digitalWrite(PRESSDOWN_PIN, HIGH);
        digitalWrite(PRESSUP_PIN, HIGH);
    }
    else if (millis() - pressTimer > PRESS_WAIT_TIME && millis() - pressTimer <= PRESS_UP_TIME) {
        digitalWrite(PRESSDOWN_PIN, LOW);
        digitalWrite(PRESSUP_PIN, HIGH);
    }
    else {
        stopPress();
    }
}

// Mesafe sensörü ile doluluk oranı hesaplanıyor
int checkOccupancy()
{
    distance = readDistance();
    if (distance == -1) {
        return -1;
    }
    else if (distance <= 300) {
        occupancy = 100;
    }
    else if (distance > 300 && distance <= 1200) {
        occupancy = map(distance, 300, 1200, 100, 0);
    }
    else {
        occupancy = 0;
    }
    return occupancy;
}

// Kapı durumu kontrol ediliyor
String checkDoorState()
{
    doorState = digitalRead(DOOR_PIN);
    if (doorState == 1)
    {
        return "Kapali";
    }
    else
    {
        return "Acik";
    }
}

// Kapak durumu kontrol ediliyor
String checkCoverState()
{
    coverState = digitalRead(COVER_PIN);
    if (coverState == 1)
    {
        return "Acik";
    }
    else
    {
        return "Kapali";
    }
}

// Manyetik kilit durumu kontrol ediliyor
String checkMagneticLockState()
{
    magneticLockState = digitalRead(MAGNETIC_LOCK_PIN);
    if (magneticLockState == 1)
    {
        return "Acik";
    }
    else
    {
        return "Kilitli";
    }
}

// Voltaj okuma fonksiyonu
float readVoltage()
{
    // return INA.getBusVoltage();
    return powerSensor.readBusVoltage();
}

// Akım okuma fonksiyonu
float readCurrent()
{
    // return INA.getCurrent();
    return powerSensor.readShuntCurrent();
}

// Güç okuma fonksiyonu
float readPower()
{
    // return INA.getPower();
    return powerSensor.readBusPower();
}

// Mesafe okuma fonksiyonu
uint16_t readDistance()
{
    VL53L0X_RangingMeasurementData_t measure;

    distanceSensor.rangingTest(&measure, false);

    if (measure.RangeStatus != 4)
    { // Mesafe ölçümü geçerliyse
        return measure.RangeMilliMeter;
    }
    else
        return -1; // Geçersiz mesafe değeri dönüşü
}

// Sıcaklık okuma fonksiyonu
float readTemperature()
{
    int reading = analogRead(TEMPERATURE_PIN);
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    float temperatureC = (voltage - 0.5) * 10;
    return temperatureC;
}

// Sıcaklık fonksiyonu
double Thermister(int RawADC) {  //Function to perform the fancy math of the Steinhart-Hart equation
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;              // Convert Kelvin to Celsius
//  Temp = (Temp * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit - comment out this line if you need Celsius
 return Temp;
}