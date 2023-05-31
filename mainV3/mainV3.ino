#include <SoftwareSerial.h> // Serial port kütüphanesi
#include <DFRobot_SIM808.h> // SIM808 kütüphanesi
#include <U8g2lib.h>          // Ekran kütüphanesi
#include <Adafruit_VL53L0X.h> // Mesafe sensörü kütüphanesi
#include <INA226.h>           // Akım ve voltaj sensörü kütüphanesi
#include <math.h>             //loads the more advanced math functions
#include <TimeLib.h>          // Zaman dönüşümleri için gerekli kütüphane

// Pinler
#define TEMPERATURE_PIN A0  // Sıcaklık sensörü
#define COVER_PIN 4         // Kapak sensörü pin
#define DOOR_PIN 5          // Kapı sensörü pin
#define MAGNETIC_LOCK_PIN 6 // Manyetik kilit pin
#define SIM808_POWER_PIN 7  // SIM808 güç pin
#define PRESSUP_PIN 8       // Presi aşağı indirmek için kullanılan pin
#define PRESSDOWN_PIN 9     // Presi yukarı çıkartmak için kullanılan pin
#define SIM808_TX_PIN 10    // SIM808 TX pin
#define SIM808_RX_PIN 11    // SIM808 RX pin

// Sabitler
#define SCREEN_WIDTH 128 // Ekran genişliği
#define SCREEN_HEIGHT 64 // Ekran yüksekliği
#define TCP_HOST "185.98.62.213"
#define TCP_PORT 8181

const long PRESS_TIME = 27000; // Pres süresi
const long PRESS_DOWN_TIME = PRESS_TIME;
const long PRESS_WAIT_TIME = PRESS_TIME + 1000;
const long PRESS_UP_TIME = (PRESS_TIME * 2) + 1000;

// Nesneler
Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();                                            // Mesafe sensörü
INA226 powerSensor;                                                                              // Akım ve voltaj sensörü
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ
SoftwareSerial mySerial(SIM808_TX_PIN, SIM808_RX_PIN);                                           // Serial port
DFRobot_SIM808 sim808(&mySerial);                                                                // SIM808

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

float voltage = 24.0;      // Voltaj değişkeni
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

bool isSystemTimeSet = false;
bool isGPSInitialized = false;
struct gspdata{
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t centisecond;
        String UTCtime;
        String localTime;
        float lat;
        float lat_dmm;
        float lon;
        float lon_dmm;
        float speed_kph;
        float speed_kmh;
        float altitude;
        float course;
    }GPSdata;

// Zaman değişkenleri
unsigned long pressCheckTime = 0;            // Pres kontrol zamanlayıcısı
unsigned long gpsCheckTime = 0;              // GPS kontrol zamanlayıcısı

// Fonksiyonlar
double Thermister(int RawADC); // Sıcaklık fonksiyonu

void setup()
{
    // Serial port başlatılıyor
    Serial.begin(9600);
    Serial.println("Starting...");

    // SIM808 başlatılıyor
    mySerial.begin(9600);

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
    pinMode(SIM808_POWER_PIN, OUTPUT);
    pinMode(PRESSDOWN_PIN, OUTPUT);
    pinMode(PRESSUP_PIN, OUTPUT);

    // Manyetik kilit varsayılan olarak açık
    digitalWrite(MAGNETIC_LOCK_PIN, HIGH);

    // Presler varsayılan olarak kapalı
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, LOW);

    // SIM808 güç pin varsayılan olarak kapalı
    digitalWrite(SIM808_POWER_PIN, LOW);

    // Pin değerleri okunuyor
    doorState = digitalRead(DOOR_PIN);
    coverState = digitalRead(COVER_PIN);
    magneticLockState = digitalRead(MAGNETIC_LOCK_PIN);

    // Mesafe sensörü başlatılıyor
    distanceSensor.begin();

    // Akım ve voltaj sensörü başlatılıyor
    powerSensor.begin();
    powerSensor.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    powerSensor.calibrate(0.01, 9);
}

void loop()
{
    updateVariables();        // Değişkenler güncelleniyor
    printVariablesToSerial(); // Değişkenler seri porta yazdırılıyor

    // Pres kontrol ediliyor
    checkPress();

    //
    if (millis() - gpsCheckTime > 5000 && !isPressWorking)
    {
        gpsCheckTime = millis();
        readGPS();
    }

    // Ekran güncelleniyor
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_u8glib_4_tr); // u8g2_font_ncenB08_tr

        u8g2.setCursor(0, 5);u8g2.print("Mesafe: ");u8g2.print(distance);u8g2.println(" mm");
        u8g2.setCursor(0, 15);u8g2.print("Voltaj: ");u8g2.print(voltage);u8g2.println(" V");
        u8g2.setCursor(0, 25);u8g2.print("Akim: ");u8g2.print(current);u8g2.println(" A");
        // u8g2.setCursor(0, 35);u8g2.print("Guc: ");u8g2.print(power);u8g2.println(" W");
        u8g2.setCursor(0, 35);u8g2.println(getLocalDateTime());
        u8g2.setCursor(0, 45);u8g2.print("Sicaklik: ");u8g2.print(temperature);u8g2.println(" C");
        u8g2.setCursor(0, 55);u8g2.print("Sicaklik2: ");u8g2.print(temperature2);u8g2.println(" C");

        u8g2.setCursor(64, 5);u8g2.print("Doluluk: ");u8g2.print(checkOccupancy());u8g2.println(" %");
        u8g2.setCursor(64, 15);u8g2.print("Kapak: ");u8g2.println(checkCoverState());
        u8g2.setCursor(64, 25);u8g2.print("Kapi: ");u8g2.println(checkDoorState());
        u8g2.setCursor(75, 35);u8g2.print("M.Kilit: ");u8g2.println(checkMagneticLockState());
        u8g2.setCursor(64, 45);u8g2.print("Pres: ");u8g2.println(checkPressState());
        // u8g2.setCursor(64, 55);u8g2.print("Pres Gerekli: ");u8g2.println(checkPressNeeded());
        u8g2.setCursor(64, 55);u8g2.print(GPSdata.lat, 4);u8g2.print(",");u8g2.print(GPSdata.lon, 4);

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

// Değişkenleri seri porta yazdırma fonksiyonu
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
}

// Presin durumunu güncelleme fonksiyonu
void updatePressState() 
{
    if (pressDownState == 0 && pressUpState == 0 && !isPressWorking && isPressReady && pressNeeded > 1 && !isPressDisabled)
    { // Pres duruyorsa ve pres hazırsa ve pres gerekliyse ve pres çalışmıyorsa ve pres devre dışı değilse
        Serial.println("Starting press...");
        pressDown();
        pressState = 0;
    }
    else if (pressDownState == 1 && pressUpState == 0 && isPressWorking && isPressReady)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println("Pressing down...");
        pressDown();
        pressState = 1;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressReady)
    { // Pres aşağı iniyorsa ve pres hazır değilse
        Serial.println("Waiting for pressing down...");
        pressWait();
        pressState = 2;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println("Continuing pressing down...");
        pressDown();
        pressState = 1;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println("Pressed down...");
        pressUp();
        pressState = 3;
    }
    else if (pressDownState == 0 && pressUpState == 1 && isPressWorking && isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println("Pressing up...");
        pressUp();
        pressState = 4;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazır değilse
        Serial.println("Waiting for pressing up...");
        pressWait();
        pressState = 5;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println("Continuing pressing up...");
        pressUp();
        pressState = 4;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println("Pressed up...");
        pressState = 6;
        pressStop();
        pressNeeded = 0;
        // if (checkPressNeeded()) {
        //     isPressDisabled = true;
        // }
    }
    else
    {
        pressState = 7;
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
    if (doorState == 1 && coverState == 0 && voltage > 19 && temperature < 70) // Kapı ve kapak kapalıysa ve voltaj 19V'dan büyükse ve sıcaklık 70C'dan küçükse
    {
        isPressReady = true;
    }
    else
    {
        isPressReady = false;
    }
    return isPressReady;
}

String checkPressState()
{
    switch (pressState)
    {
    case 0:
        return "Baslatiliyor";
    case 1:
        return "Asagi iniyor";
    case 2:
        return "Asagi beklemede";
    case 3:
        return "Asagida";
    case 4:
        return "Yukari cikiyor";
    case 5:
        return "Yukari beklemede";
    case 6:
        return "Yukarida";
    case 7:
        return "Duruyor";
    default:
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
    digitalWrite(MAGNETIC_LOCK_PIN, LOW);
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, LOW);
    pressDownState = 1;
    pressUpState = 0;
    isPressPaused = false;
}

// Presi yukarı çıkartma fonksiyonu
void pressUp()
{
    digitalWrite(MAGNETIC_LOCK_PIN, LOW);
    digitalWrite(PRESSDOWN_PIN, LOW);
    digitalWrite(PRESSUP_PIN, HIGH);
    pressDownState = 0;
    pressUpState = 1;
    isPressPaused = false;
}

// Presi bekletme fonksiyonu
void pressWait()
{
    digitalWrite(MAGNETIC_LOCK_PIN, HIGH);
    digitalWrite(PRESSDOWN_PIN, HIGH);
    digitalWrite(PRESSUP_PIN, HIGH);
    isPressPaused = true;
}

// Presi durdurma fonksiyonu
void pressStop()
{
    digitalWrite(MAGNETIC_LOCK_PIN, HIGH);
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

// Zaman fonksiyonu
String getLocalDateTime()
{
    String dateTime = "";
    dateTime += String(year());
    dateTime += "/";
    dateTime += getDigits(month());
    dateTime += "/";
    dateTime += getDigits(day());
    dateTime += " ";
    dateTime += getDigits(hour());
    dateTime += ":";
    dateTime += getDigits(minute());
    dateTime += ":";
    dateTime += getDigits(second());
       
    return dateTime;
}

// Sayıyı 2 basamaklı stringe çevirme fonksiyonu
String getDigits(int digits)
{
    String digitsStr = "";
    if(digits < 10)
        digitsStr += '0';
    digitsStr += String(digits);
    return digitsStr;
}

// SIM808 açma kapama fonksiyonu
void powerUpDownSIM808()
{
    Serial.println("Resetting SIM808...");
    digitalWrite(SIM808_POWER_PIN, HIGH);
    delay(3000);
    digitalWrite(SIM808_POWER_PIN, LOW);
    powerUpGPS();
}

// SIM808 GPS açma fonksiyonu
void powerUpGPS()
{
    Serial.println("Powering up GPS...");
    if (sim808_check_with_cmd("AT+CGPSPWR=1\r\n", "OK\r\n", CMD)) // GPS'i açma
    {
        Serial.println("GPS powered up!");
        isGPSInitialized = true;
    }
    else
    {
        Serial.println("GPS power up failed!");
        isGPSInitialized = false;
    }
    // delay(100);
}

String convertToLocalDate(String time)
{
    // UTC formatındaki zamanı yerel saat formatına dönüştürme
    String year = time.substring(0, 4);
    String month = time.substring(4, 6);
    String day = time.substring(6, 8);
    String hour = time.substring(8, 10);
    String minute = time.substring(10, 12);
    String second = time.substring(12, 14);
    if (!isSystemTimeSet && year.toInt() > 2022) {
        setTime(hour.toInt() + 3, minute.toInt(), second.toInt(), day.toInt(), month.toInt(), year.toInt());
        isSystemTimeSet = true;
    }
    String localDate = String(day) + "/" + String(month) + "/" + String(year);
    String localTime = String(hour) + ":" + String(minute) + ":" + String(second);
    return localDate + " " + localTime;
}

float convertToKilometersPerHour(String kphValue) {
    // KPH formatındaki değeri KM/H formatına dönüştürme
    float kph = kphValue.toFloat();
    float kmh = kph * 1.852;
    return kmh;
}

void parseGPSData(String gpsData) {
    // İstenmeyen kısmı atlayarak veriyi parçalara bölme
    int startIndex = gpsData.indexOf(":") + 1;
    String data = gpsData.substring(startIndex);

    // Parçaları virgül karakterine göre ayırma
    int commaIndex = data.indexOf(",");
    String mode = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String latitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String longitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String altitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String UTCTime = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String TTFF = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String num = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(",");
    String speed = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    String course = data;

    // GPSData struct'ına verileri kaydetme
    GPSdata.year = UTCTime.substring(0, 4).toInt();
    GPSdata.month = UTCTime.substring(4, 6).toInt();
    GPSdata.day = UTCTime.substring(6, 8).toInt();
    GPSdata.hour = UTCTime.substring(8, 10).toInt();
    GPSdata.minute = UTCTime.substring(10, 12).toInt();
    GPSdata.second = UTCTime.substring(12, 14).toInt();
    GPSdata.centisecond = UTCTime.substring(14, 16).toInt();
    GPSdata.UTCtime = UTCTime;
    GPSdata.lat_dmm = latitude.toFloat();
    GPSdata.lon_dmm = longitude.toFloat();
    GPSdata.altitude = altitude.toFloat();
    GPSdata.speed_kph = speed.toFloat();
    GPSdata.course = course.toFloat();

    // Dönüşümleri yapma        
    GPSdata.lat = convertToDecimalDegrees(latitude);
    GPSdata.lon = convertToDecimalDegrees(longitude);
    GPSdata.speed_kmh = convertToKilometersPerHour(speed);
    GPSdata.localTime = convertToLocalDate(UTCTime);

    // Elde edilen verileri yazdırma
    // Serial.println("Mode: " + mode);
    // Serial.println("Latitude (dd): " + String(GPSdata.lat, 6));
    // Serial.println("Longitude (dd): " + String(GPSdata.lon, 6));
    // Serial.println("Altitude: " + altitude);
    // Serial.println("UTC Time: " + UTCTime);
    // Serial.println("Local Time: " + GPSdata.localTime);
    // Serial.println("TTFF: " + TTFF);
    // Serial.println("Num: " + num);
    // Serial.println("Speed (km/h): " + String(GPSdata.speed_kmh, 2));
    // Serial.println("Course: " + course);
}

float convertToDecimalDegrees(String dmmValue) {
    // DMM formatındaki değeri ondalık dereceye dönüştürme
    int degrees = dmmValue.substring(0, 2).toInt();
    float minutes = dmmValue.substring(2).toFloat();
    float decimalDegrees = degrees + (minutes / 60.0);
    return decimalDegrees;
}

String readGPS()
{
    if (!isGPSInitialized)
    {
        powerUpGPS();
    }
    String gpsData;

    gpsData = readSIM808("AT+CGPSINF=0\r\n", "CGPSINF: ");
    // Serial.println("GPS DATA: ");
    // Serial.println(gpsData);
    parseGPSData(gpsData);
    return gpsData;
}

String readSIM808(String cmd, String resp)
{
    String data = "";
    sim808_send_cmd(cmd.c_str());
    int i = 0;
    do {
        data = mySerial.readStringUntil('\n');
        // Serial.print("Read: ");Serial.println(i++);
        // Serial.println(data);
        if (strstr(data.c_str(), resp.c_str()) != NULL)
        {
            // Serial.println("strstr(data, resp) != NULL");
            return data;
        }
    } while (data != "");
    return data;
}