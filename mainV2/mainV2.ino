// #include <SoftwareSerial.h> // SIM808 kütüphanesi
#include <U8g2lib.h>          // Ekran kütüphanesi
#include <Adafruit_VL53L0X.h> // Mesafe sensörü kütüphanesi
#include <INA226.h>           // Akım ve voltaj sensörü kütüphanesi
#include <math.h>             //loads the more advanced math functions

// #include "pressClass.h" // Pres kütüphanesi

// Pinler
#define TEMPERATURE_PIN A0  // Sıcaklık sensörü
#define COVER_PIN 4         // Kapak sensörü pin
#define DOOR_PIN 5          // Kapı sensörü pin
#define MAGNETIC_LOCK_PIN 6 // Manyetik kilit pin
#define PRESSUP_PIN 8       // Presi aşağı indirmek için kullanılan pin
#define PRESSDOWN_PIN 9     // Presi yukarı çıkartmak için kullanılan pin
#define SIM808_RX_PIN 10    // SIM808 RX pin
#define SIM808_TX_PIN 11    // SIM808 TX pin

// Sabitler
#define SCREEN_WIDTH 128 // Ekran genişliği
#define SCREEN_HEIGHT 64 // Ekran yüksekliği

const long PRESS_TIME = 27000; // Pres süresi
const long PRESS_DOWN_TIME = PRESS_TIME;
const long PRESS_WAIT_TIME = PRESS_TIME + 1000;
const long PRESS_UP_TIME = (PRESS_TIME * 2) + 1000;

// Nesneler
Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();                                            // Mesafe sensörü
INA226 powerSensor;                                                                              // Akım ve voltaj sensörü
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ

// pressClass myPress(PRESSDOWN_PIN); // Pres nesnesi

// Değişkenler
int doorState = 0;         // Kapı durumu
int coverState = 0;        // Kapak durumu
int magneticLockState = 0; // Manyetik kilit durumu
int occupancy = 0;         // Doluluk oranı
int distance = 0;          // Mesafe değişkeni
int pressNeeded = 0;       // Presin gerekli olup olmadığı durumu
int pressState = 0;        // Pres durumu
int pressDownState = 0;    // Presin aşağı inme durumu
int pressUpState = 1;      // Presin yukarı çıkma durumu

float voltage = 0.0;      // Voltaj değişkeni
float current = 0.0;      // Akım değişkeni
float power = 0.0;        // Güç değişkeni
float temperature = 0.0;  // Sıcaklık değişkeni
float temperature2 = 0.0; // Sıcaklık değişkeni 2

bool isPressing = false;      // Pres durumu
bool isPressReady = false;    // Pres hazır durumu
bool isPressCentered = false; // Presin ortalanma durumu
bool isPressWorking = false;  // Presin çalışma durumu
bool isPressPaused = true;   // Presin durdurulma durumu
bool isPressDisabled = false; // Presin devre dışı bırakılma durumu

// Zaman değişkenleri
unsigned long pressTimer = 0;       // Pres zamanlayıcısı
unsigned long pressCenteringStartedTime = 0; // Presin ortalanma zamanlayıcısı
unsigned long pressCheckTime = 0;            // Pres kontrol zamanlayıcısı

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
    u8g2.drawStr((SCREEN_WIDTH / 2) - (w / 2), 32, startingText.c_str());
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
    powerSensor.begin();
    powerSensor.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    powerSensor.calibrate(0.01, 9);
}

void loop()
{
    // Serial.print("Running: ");
    // Serial.println(millis());

    // distance = readDistance();
    // // Serial.print("Distance: ");
    // // Serial.println(distance);

    // occupancy = checkOccupancy();
    // // Serial.print("Occupancy: ");
    // // Serial.println(occupancy);

    // voltage = readVoltage();
    // // Serial.print("Voltage: ");
    // // Serial.println(voltage);

    // current = readCurrent();
    // // Serial.print("Current: ");
    // // Serial.println(current);

    // power = readPower();
    // // Serial.print("Power: ");
    // // Serial.println(power);

    // temperature = readTemperature();
    // // Serial.print("Temperature: ");
    // // Serial.println(temperature);

    // temperature2 = Thermister(analogRead(TEMPERATURE_PIN));
    // // Serial.print("Temperature2: ");
    // // Serial.println(temperature2);

    updateVariables();        // Değişkenler güncelleniyor
    printVariablesToSerial(); // Değişkenler seri porta yazdırılıyor

    // Pres kontrol ediliyor
    checkPress();

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

        u8g2.setCursor(60, 60);
        u8g2.print("Pres Gerekli: ");
        u8g2.println(checkPressNeeded());

    } while (u8g2.nextPage());
}

// Değişkenleri güncelleme fonksiyonu
void updateVariables()
{
    // Pin değerleri okunuyor
    doorState = digitalRead(DOOR_PIN);
    coverState = digitalRead(COVER_PIN);
    magneticLockState = digitalRead(MAGNETIC_LOCK_PIN);

    // Mesafe sensörü okunuyor
    distance = readDistance();

    // Doluluk oranı hesaplanıyor
    occupancy = checkOccupancy();

    // Akım ve voltaj sensörü okunuyor
    voltage = readVoltage();
    current = readCurrent();
    power = readPower();

    // Sıcaklık sensörü okunuyor
    temperature = readTemperature();
    temperature2 = Thermister(analogRead(TEMPERATURE_PIN));
}

// Değişkenleri yazdırma fonksiyonu
void printVariablesToSerial()
{
    Serial.print("Distance: ");Serial.print(distance);Serial.print("mm");Serial.print(" | ");
    Serial.print("Voltage: ");Serial.print(voltage);Serial.print("V");Serial.print(" | ");
    Serial.print("Current: ");Serial.print(current);Serial.print("A");Serial.print(" | ");
    Serial.print("Power: ");Serial.print(power);Serial.print("W");Serial.print(" | ");
    Serial.print("Temperature: ");Serial.print(temperature);Serial.print("C");Serial.print(" | ");
    Serial.print("Temperature2: ");Serial.print(temperature2);Serial.print("C");Serial.print(" | ");
    Serial.print("Occupancy: ");Serial.print(occupancy);Serial.print("%");Serial.print(" | ");
    Serial.print("Cover: ");Serial.print(checkCoverState());Serial.print(" | ");
    Serial.print("Door: ");Serial.print(checkDoorState());Serial.print(" | ");
    Serial.print("Magnetic Lock: ");Serial.print(checkMagneticLockState());
    Serial.println();
}

// Pres kontrol fonksiyonu
void checkPress()
{
    checkPressWorking(); // Presin çalışıp çalışmadığı kontrol ediliyor
    checkPressReady();   // Presin hazır olup olmadığı kontrol ediliyor
    // if (!isPressCentered)
    // {
    //     if (isPressReady)
    //     {
    //         Serial.println("Centering press...");
    //         pressUp();
    //         if (pressCenteringStartedTime == 0)
    //         {
    //             pressCenteringStartedTime = millis();
    //         }
    //         else if (millis() - pressCenteringStartedTime > 1000)
    //         {
    //             isPressCentered = !isPressWorking && !isPressPaused;
    //         }
    //     }
    //     else
    //     {
    //         pressWait();
    //     }
    // }
    // else
    // {
    //     if (millis() - pressCheckTime > 1000)
    //     {
    //         pressCheckTime = millis();
    //         if (checkPressNeeded())
    //         {
    //             pressNeeded++;
    //         }
    //         else
    //         {
    //             pressNeeded = 0;
    //         }
    //     }
    //     updatePressState(); // Pres durumu güncelleniyor
    // }

    if (millis() - pressCheckTime > 1000)
    {
        pressCheckTime = millis();
        if (checkPressNeeded())
        {
            pressNeeded++;
        }
        else
        {
            pressNeeded = 0;
        }
    }
    updatePressState(); // Pres durumu güncelleniyor
    // if (!isPressCentered) {
    //     isPressCentered = true;
    // } else {
    //     updatePressState(); // Pres durumu güncelleniyor
    // }

}

// Presin durumunu güncelleme fonksiyonu
void updatePressState() 
{
    if (pressDownState == 0 && pressUpState == 0 && !isPressWorking && isPressReady && pressNeeded > 1 && !isPressDisabled)
    { // Pres duruyorsa ve pres hazırsa ve pres gerekliyse ve pres çalışmıyorsa ve pres devre dışı değilse
        Serial.println("Starting press...");
        pressDown();
    }
    else if (pressDownState == 1 && pressUpState == 0 && isPressWorking && isPressReady)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println("Pressing down...");
        pressDown();
    }
    else if (pressDownState == 1 && pressUpState == 0 && isPressWorking && !isPressReady)
    { // Pres aşağı iniyorsa ve pres hazır değilse ve pres çalışıyorsa
        Serial.println("Waiting for pressing down...");
        pressWait();
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println("Continuing pressing down...");
        pressDown();
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println("Pressed down...");
        pressUp();
    }
    else if (pressDownState == 0 && pressUpState == 1 && isPressWorking && isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println("Pressing up...");
        pressUp();
    }
    else if (pressDownState == 0 && pressUpState == 1 && isPressWorking && !isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazır değilse ve pres çalışıyorsa
        Serial.println("Waiting for pressing up...");
        pressWait();
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println("Continuing pressing up...");
        pressUp();
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println("Pressed up...");
        pressStop();
        pressNeeded = 0;
        // if (checkPressNeeded()) {
        //     isPressDisabled = true;
        // }
    }
    else
    {
        Serial.println("Stopping press...!!!");
    }
}


// Presin gerekli olup olmadığını kontrol eden fonksiyon
bool checkPressNeeded()
{
    occupancy = checkOccupancy();
    return occupancy >= 90;
}

// Pres yapmaya hazır mı kontrol eden fonksiyon
bool checkPressReady()
{
    if (doorState == 1 && coverState == 0)
    {
        isPressReady = true;
    }
    else
    {
        isPressReady = false;
    }
    return isPressReady;
}

// Pres durumunu kontrol eden fonksiyon
String checkPressState()
{
    if (pressDownState == 1 && pressUpState == 0 && checkPressWorking() && checkPressReady())
    {
        pressState = 1;
        return "Asagi iniyor";
    }
    else if (pressDownState == 1 && pressUpState == 0 && !checkPressWorking() && !checkPressReady())
    {
        pressState = 2;
        return "Asagi beklemede";
    }
    else if (pressDownState == 1 && pressUpState == 0 && !checkPressWorking() && checkPressReady())
    {
        pressState = 3;
        return "Asagida";
    }
    else if (pressDownState == 0 && pressUpState == 1 && checkPressWorking() && checkPressReady())
    {
        pressState = 4;
        return "Yukari cikiyor";
    }
    else if (pressDownState == 0 && pressUpState == 1 && !checkPressWorking() && !checkPressReady())
    {
        pressState = 5;
        return "Yukari beklemede";
    }
    else if (pressDownState == 0 && pressUpState == 1 && !checkPressWorking() && checkPressReady())
    {
        pressState = 6;
        return "Yukarida";
    }
    else if (pressDownState == 1 && pressUpState == 1 && !checkPressWorking() && checkPressReady())
    {
        pressState = 7;
        return "Beklemede";
    }
    else if (pressDownState == 1 && pressUpState == 1 && !checkPressWorking() && !checkPressReady())
    {
        pressState = 8;
        return "Kontrolde";
    }
    // else if (pressDownState == 0 && pressUpState == 0 && !checkPressWorking()) {
    //     pressState = 0;
    //     return "Durdu";
    // }
    else
    {
        pressState = 0;
        return "Kapali";
    }
}

// Presin çalışıp çalışmadığını kontrol eden fonksiyon
bool checkPressWorking()
{
    float currentValue = readCurrent();
    isPressWorking = currentValue > 4.0;
    return isPressWorking;
}

// Presi aşağı indirme fonksiyonu
void pressDown()
{
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, LOW);
    pressDownState = 1;
    pressUpState = 0;
    isPressPaused = false;
}

// Presi yukarı çıkartma fonksiyonu
void pressUp()
{
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, HIGH);
    pressDownState = 0;
    pressUpState = 1;
    isPressPaused = false;
}

// Presi bekletme fonksiyonu
void pressWait()
{
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, HIGH);
    // pressDownState = 1;
    // pressUpState = 1;
    isPressPaused = true;
}

// Presi durdurma fonksiyonu
void pressStop()
{
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);
    pressDownState = 0;
    pressUpState = 0;
    isPressPaused = false;
}

// Mesafe sensörü ile doluluk oranı hesaplanıyor
int checkOccupancy()
{
    // distance = readDistance();
    const int fullDistance = 400;
    const int emptyDistance = 1300;
    const int distanceJump = (emptyDistance - fullDistance) / 20;

    switch (distance)
    {
    case 0 ... fullDistance:
        return 100;
    case fullDistance + 1 ... emptyDistance:
        return 100 - 5 * ((distance - fullDistance) / distanceJump);
    default:
        return 0;
    }
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
    return powerSensor.readBusVoltage();
}

// Akım okuma fonksiyonu
float readCurrent()
{
    return powerSensor.readShuntCurrent();
}

// Güç okuma fonksiyonu
float readPower()
{
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
double Thermister(int RawADC)
{ // Function to perform the fancy math of the Steinhart-Hart equation
    double Temp;
    Temp = log(((10240000 / RawADC) - 10000));
    Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp)) * Temp);
    Temp = Temp - 273.15; // Convert Kelvin to Celsius
    // Temp = (Temp * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit - comment out this line if you need Celsius
    return Temp;
}