// Kütüphaneler
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

#define PRESS_CHECK_TIME 30000 // Pres kontrol zamanlayıcısı
#define SIM808_CHECK_TIME 60000 // GPS kontrol zamanlayıcısı
#define PRESS_WORKING_CURRENT 0.5 // Presin çalışma akımı

#define MIN_OCCUPANCY_DISTANCE 400 // Minimum doluluk mesafesi
#define MAX_OCCUPANCY_DISTANCE 1300 // Maksimum doluluk mesafesi
#define OCCUPANCY_DISTANCE_JUMP 45 // Doluluk mesafesi artışı
#define MIN_WORKING_VOLTAGE 19 // Minimum çalışma voltajı
#define MAX_WORKING_TEMPERATURE 50 // Maksimum çalışma sıcaklığı

// Nesneler
Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();                                            // Mesafe sensörü
INA226 powerSensor;                                                   // Akım ve voltaj sensörü
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ
SoftwareSerial mySerial(SIM808_TX_PIN, SIM808_RX_PIN);                                           // Serial port
DFRobot_SIM808 sim808(&mySerial);                                                                // SIM808

// Değişkenler
int occupancy = 0;         // Doluluk oranı
int distance = 0;          // Mesafe değişkeni
int pressNeeded = 0;       // Presin gerekli olup olmadığı durumu
int pressDownState = 0;    // Presin aşağı inme durumu
int pressUpState = 1;      // Presin yukarı çıkma durumu

float voltage = 24.0;      // Voltaj değişkeni
float current = 0.0;      // Akım değişkeni
float power = 0.0;        // Güç değişkeni
float temperature = 0.0;  // Sıcaklık değişkeni

bool isPressing = false;      // Pres durumu
bool isPressReady = false;    // Pres hazır durumu
bool isPressCentered = false; // Presin ortalanma durumu
bool isPressWorking = false;  // Presin çalışma durumu
bool isPressPaused = true;   // Presin durdurulma durumu
bool isPressDisabled = false; // Presin devre dışı bırakılma durumu
bool isSystemTimeSet = false; // Sistemin zamanının ayarlanma durumu
bool isGPSInitialized = false; // GPS'in başlatılma durumu

// GPS verileri
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

// SIM808 durumları
enum SIM808_STATES {
    SIM808_POWER_DOWN,
    SIM808_POWER_UP,
    SIM808_SIM_NOT_READY,
    SIM808_SIM_READY,
    SIM808_INITIALIZED,
    SIM808_JOINING_TO_NETWORK,
    SIM808_JOINED_TO_NETWORK,
    SIM808_TCP_NOT_CONNECTED,
    SIM808_TCP_CONNECTED,
    SIM808_REGISTERED_TO_SERVER,
};

// Pres durumları
enum PRESS_STATES {
    PRESS_STOPPED,
    PRESS_STARTING,
    PRESSED_DOWN,
    PRESSED_UP,
    PRESS_DOWN_WAITING,
    PRESS_UP_WAITING,
    PRESS_IS_GOING_DOWN,
    PRESS_IS_GOING_UP,
};

// Kapı durumları
enum DOOR_STATES {
    DOOR_CLOSED,
    DOOR_OPENED,
};

// Kapak durumları
enum COVER_STATES {
    COVER_CLOSED,
    COVER_OPENED,
};

// Manyetik kilit durumları
enum MAGNETIC_LOCK_STATES {
    MAGNETIC_LOCKED,
    MAGNETIC_UNLOCKED,
};

// Ekranda ne yazacağını belirleyen durumlar
enum DISPLAY_STATES {
    DOOR_AND_COVER_CLOSED,
    DOOR_CLOSED_COVER_OPENED,
    DOOR_OPENED_COVER_CLOSED,
    DOOR_AND_COVER_OPENED,
    PRESS_WORKING,
};

enum SIM808_STATES sim808State = SIM808_POWER_DOWN; // SIM808 durumu
enum PRESS_STATES pressState = PRESS_STOPPED;       // Pres durumu
enum DOOR_STATES doorState = DOOR_CLOSED;         // Kapı durumu
enum COVER_STATES coverState = COVER_CLOSED;        // Kapak durumu
enum MAGNETIC_LOCK_STATES magneticLockState = MAGNETIC_UNLOCKED; // Manyetik kilit durumu
char buffer[1024];

// Zaman değişkenleri
unsigned long pressCheckTime = 0;            // Pres kontrol zamanlayıcısı
unsigned long sim808CheckTime = 0;              // GPS kontrol zamanlayıcısı

// Fonksiyonlar
void setup(); // Setup fonksiyonu
void loop();  // Loop fonksiyonu
void updateVariables(); // Değişkenleri güncelleme fonksiyonu
void printVariablesToSerial(); // Değişkenleri seri porta yazdırma fonksiyonu
void checkPress(); // Pres kontrol fonksiyonu
void updatePressState(); // Presin durumunu güncelleme fonksiyonu
bool checkPressNeeded(); // Presin gerekli olup olmadığını kontrol eden fonksiyon
bool checkPressReady(); // Pres yapmaya hazır mı kontrol eden fonksiyon
String checkPressState(); // Pres durumu kontrol ediliyor
bool checkPressWorking(); // Presin çalışıp çalışmadığını kontrol eden fonksiyon
void pressDown(); // Presi aşağı indirme fonksiyonu
void pressUp(); // Presi yukarı çıkartma fonksiyonu
void pressWait(); // Presi bekletme fonksiyonu
void pressStop(); // Presi durdurma fonksiyonu
void checkOccupancy(); // Mesafe sensörü ile doluluk oranı hesaplanıyor
String getDoorState(); // Kapı durumu kontrol ediliyor
String getCoverState(); // Kapak durumu kontrol ediliyor
String getMagneticLockState(); // Manyetik kilit durumu kontrol ediliyor
float readVoltage(); // Voltaj okuma fonksiyonu
float readCurrent(); // Akım okuma fonksiyonu
float readPower(); // Güç okuma fonksiyonu
uint16_t readDistance(); // Mesafe okuma fonksiyonu
float readTemperature(); // Sıcaklık okuma fonksiyonu
String getLocalDateTime(); // Zaman fonksiyonu
String getDigits(const int number); // Sayıyı 2 basamaklı hale getirme fonksiyonu
void powerUpDownSIM808(); // SIM808 güç açma kapama fonksiyonu
bool powerUpGPS(); // GPS güç açma fonksiyonu
String convertToLocalDate(const  String time); // UTC zamanı yerel zamana çevirme fonksiyonu
float convertToKilometersPerHour(const String kphValue); // Hızı km/saat cinsinden çevirme fonksiyonu
void parseGPSData(const String gpsData); // GPS verilerini ayrıştırma fonksiyonu
float convertToDecimalDegrees(const String dmmValue); // DMM cinsinden koordinatı ondalık dereceye çevirme fonksiyonu
void readGPS(); // GPS verilerini okuma fonksiyonu
String readSIM808(const String cmd, const String resp); // SIM808 komutu okuma fonksiyonu
String getSIM808State(); // SIM808 durumunu döndüren fonksiyon
void sendSystemStateToServer(); // Sistem durumu sunucuya gönderiliyor
String getSystemState(); // Sistem durumunu döndüren fonksiyon
void checkSIM808Status(); // SIM808 durumu kontrol ediliyor
String getIMEI(); // IMEI numarasını döndüren fonksiyon
void printToDisplayScreen(const int x, const int y, const String text); // Ekrana yazı yazdırma fonksiyonu
String recvFromServer(); // Sunucudan veri okuma fonksiyonu
String checkAvailableMessages(); // Sunucudan gelen verileri okuma fonksiyonu

void setup()
{
    // Serial port başlatılıyor
    Serial.begin(9600);
    Serial.println();
    Serial.println(F("Starting..."));

    // SIM808 başlatılıyor
    Serial.println(F("Starting MySerial..."));
    mySerial.begin(9600);

    // Ekran başlatılıyor
    const String startingText = "BASLATILIYOR...";
    Serial.println(F("Starting display..."));
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    const int w = u8g2.getUTF8Width(startingText.c_str());
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

    // Sistem başlatılıyor
    delay(1000);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_u8glib_4_tr); // u8g2_font_ncenB08_tr   
    printToDisplayScreen(0, 5, F("Sistem Baslatiliyor..."));

    // Mesafe sensörü başlatılıyor
    Serial.println(F("Distance Module..."));
    printToDisplayScreen(0, 15, F("Mesafe Modulu:"));
    if (!distanceSensor.begin())
    {
        Serial.println(F("Distance Module Failed!"));
        printToDisplayScreen(64, 15, F("Baslatilamadi!"));
    } else {
        Serial.println(F("Distance Module Started."));
        printToDisplayScreen(64, 15, F("Baslatildi."));
    }

    // Akım ve voltaj sensörü başlatılıyor
    Serial.println(F("Starting Power Module..."));
    printToDisplayScreen(0, 25, F("Guc Modulu:"));
    if (powerSensor.begin())
    {
        Serial.println(F("Power Module Started."));
        printToDisplayScreen(64, 25, F("Baslatildi."));
        powerSensor.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
        // powerSensor.calibrate(0.1, 10);
        powerSensor.calibrate();
    } else {
        Serial.println(F("Power Module Failed."));
        printToDisplayScreen(64, 25, F("Baslatilamadi!"));
    }
    
    // Sim808 başlatılıyor
    Serial.println(F("Starting SIM808..."));
    printToDisplayScreen(0, 35, F("SIM808:"));
    if (sim808.checkPowerUp())
    {
        Serial.println(F("SIM808 Started."));
        printToDisplayScreen(64, 35, F("Baslatildi."));
        sim808State = SIM808_POWER_UP;
    } else {
        Serial.println(F("SIM808 Failed!"));
        printToDisplayScreen(64, 35, F("Baslatilamadi!"));
        sim808State = SIM808_POWER_DOWN;
    }

    // SIM808 GPS başlatılıyor
    Serial.println(F("Starting GPS..."));
    printToDisplayScreen(0, 45, F("GPS:"));
    if (powerUpGPS()) // GPS'i açma
    {
        Serial.println(F("GPS Started."));
        printToDisplayScreen(64, 45, F("Baslatildi."));
        sim808State = SIM808_POWER_UP;
    } else {
        Serial.println(F("GPS Failed!"));
        printToDisplayScreen(64, 45, F("Baslatilamadi!"));
        sim808State = SIM808_POWER_DOWN;
    }

    printToDisplayScreen(0, 55, F("Sistem Baslatildi."));
    Serial.println(F("System Started."));
    delay(5000);
}

void loop()
{
    updateVariables();        // Değişkenler güncelleniyor
    printVariablesToSerial(); // Değişkenler seri porta yazdırılıyor

    // Pres kontrol ediliyor
    checkPress();

    // Gelen veriler okunuyor
    const String recvData = checkAvailableMessages(); // Sunucudan gelen veriler okunuyor
    if (recvData != "")
    {
        Serial.println(F("Received data:"));
        Serial.println(recvData);
    }

    // SIM808 kontrol ediliyor
    if (millis() - sim808CheckTime > SIM808_CHECK_TIME && !isPressWorking)
    {
        sim808CheckTime = millis();
        readGPS();
        checkSIM808Status();
        sendSystemStateToServer();
    }

    // Ekran güncelleniyor
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_u8glib_4_tr); // u8g2_font_ncenB08_tr

        u8g2.setCursor(0, 5);u8g2.print(F("Mesafe: "));u8g2.print(distance);u8g2.println(F(" mm"));
        u8g2.setCursor(0, 15);u8g2.print(F("Voltaj: "));u8g2.print(voltage);u8g2.println(F(" V"));
        u8g2.setCursor(0, 25);u8g2.print(F("Akim: "));u8g2.print(current);u8g2.println(F(" A"));
        u8g2.setCursor(0, 35);u8g2.println(getLocalDateTime());
        u8g2.setCursor(0, 45);u8g2.print(F("Sicaklik: "));u8g2.print(temperature);u8g2.println(F(" C"));
        u8g2.setCursor(0, 55);u8g2.print(F("SIM: "));u8g2.println(getSIM808State());

        u8g2.setCursor(64, 5);u8g2.print(F("Doluluk: "));u8g2.print(occupancy);u8g2.println(F(" %"));
        u8g2.setCursor(64, 15);u8g2.print(F("Kapak: "));u8g2.println(getCoverState());
        u8g2.setCursor(64, 25);u8g2.print(F("Kapi: "));u8g2.println(getDoorState());
        u8g2.setCursor(75, 35);u8g2.print(F("M.Kilit: "));u8g2.println(getMagneticLockState());
        u8g2.setCursor(64, 45);u8g2.print(F("Pres: "));u8g2.println(checkPressState());
        // u8g2.setCursor(64, 55);u8g2.print(F("Pres Gerekli: "));u8g2.println(checkPressNeeded());
        u8g2.setCursor(64, 55);u8g2.print(GPSdata.lat, 4);u8g2.print(F(","));u8g2.print(GPSdata.lon, 4);

    } while (u8g2.nextPage());
}

// Değişkenleri güncelleme fonksiyonu
void updateVariables()
{
    // Pin değerleri okunuyor
    checkDoorState();
    checkCoverState();
    checkMagneticLockState();

    // Mesafe sensörü okunuyor
    distance = readDistance();

    // Doluluk oranı hesaplanıyor
    checkOccupancy();

    // Akım ve voltaj sensörü okunuyor
    voltage = readVoltage();
    current = readCurrent();
    power = readPower();

    // Sıcaklık sensörü okunuyor
    temperature = readTemperature();
}

// Değişkenleri seri porta yazdırma fonksiyonu
void printVariablesToSerial()
{
    Serial.println(getLocalDateTime());
    Serial.print(F("Distance: "));Serial.print(distance);Serial.print(F("mm"));Serial.print(F(" | "));
    Serial.print(F("Voltage: "));Serial.print(voltage);Serial.print(F("V"));Serial.print(F(" | "));
    Serial.print(F("Current: "));Serial.print(current);Serial.print(F("A"));Serial.print(F(" | "));
    Serial.print(F("Power: "));Serial.print(power);Serial.print(F("W"));Serial.print(F(" | "));
    Serial.print(F("Temperature: "));Serial.print(temperature);Serial.print(F("C"));Serial.print(F(" | "));
    Serial.print(F("Occupancy: "));Serial.print(occupancy);Serial.print(F("%"));Serial.print(F(" | "));
    Serial.print(F("Cover: "));Serial.print(getCoverState());Serial.print(F(" | "));
    Serial.print(F("Door: "));Serial.print(getDoorState());Serial.print(F(" | "));
    Serial.print(F("Magnetic Lock: "));Serial.print(getMagneticLockState());Serial.print(F(" | "));
    Serial.print(F("Press: "));Serial.print(checkPressState());Serial.print(F(" | "));
    Serial.print(F("SIM: "));Serial.print(getSIM808State());
    Serial.println();
}

// Pres kontrol fonksiyonu
void checkPress()
{
    checkPressWorking(); // Presin çalışıp çalışmadığı kontrol ediliyor
    checkPressReady();   // Presin hazır olup olmadığı kontrol ediliyor

    if (millis() - pressCheckTime > PRESS_CHECK_TIME && !isPressWorking)
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
        Serial.println(F("Starting press..."));
        pressDown();
        pressState = PRESS_STARTING;
    }
    else if (pressDownState == 1 && pressUpState == 0 && isPressWorking && isPressReady)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println(F("Pressing down..."));
        pressDown();
        pressState = PRESS_IS_GOING_DOWN;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressReady)
    { // Pres aşağı iniyorsa ve pres hazır değilse
        Serial.println(F("Waiting for pressing down..."));
        pressWait();
        pressState = PRESS_DOWN_WAITING;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println(F("Continuing pressing down..."));
        pressDown();
        pressState = PRESS_IS_GOING_DOWN;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println(F("Pressed down..."));
        pressUp();
        pressState = PRESSED_DOWN;
    }
    else if (pressDownState == 0 && pressUpState == 1 && isPressWorking && isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışıyorsa
        Serial.println(F("Pressing up..."));
        pressUp();
        pressState = PRESS_IS_GOING_UP;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazır değilse
        Serial.println(F("Waiting for pressing up..."));
        pressWait();
        pressState = PRESS_UP_WAITING;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        Serial.println(F("Continuing pressing up..."));
        pressUp();
        pressState = PRESS_IS_GOING_UP;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        Serial.println(F("Pressed up..."));
        pressState = PRESSED_UP;
        pressStop();
        pressNeeded = 0;
    }
    else
    {
        pressState = PRESS_STOPPED;
    }
}

// Presin gerekli olup olmadığını kontrol eden fonksiyon
bool checkPressNeeded()
{
    return occupancy >= 90;
}

// Pres yapmaya hazır mı kontrol eden fonksiyon
bool checkPressReady()
{
    if (doorState == DOOR_CLOSED && coverState == COVER_CLOSED && voltage > MIN_WORKING_VOLTAGE && temperature < MAX_WORKING_TEMPERATURE) // Kapı ve kapak kapalıysa ve voltaj 19V'dan büyükse ve sıcaklık 50C'dan küçükse
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
    case PRESS_STARTING:
        return F("Baslatiliyor");
    case PRESS_IS_GOING_DOWN:
        return F("Asagi iniyor");
    case PRESS_DOWN_WAITING:
        return F("Asagi beklemede");
    case PRESSED_DOWN:
        return F("Asagida");
    case PRESS_IS_GOING_UP:
        return F("Yukari cikiyor");
    case PRESS_UP_WAITING:
        return F("Yukari beklemede");
    case PRESSED_UP:
        return F("Yukarida");
    case PRESS_STOPPED:
        return F("Duruyor");
    default:
        return F("Kapali");
    }
}

// Presin çalışıp çalışmadığını kontrol eden fonksiyon
bool checkPressWorking()
{
    isPressWorking = readCurrent() > PRESS_WORKING_CURRENT;
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
void checkOccupancy()
{
    switch (distance)
    {
    case 0 ... MIN_OCCUPANCY_DISTANCE:
        occupancy = 100;
        break;
    case MIN_OCCUPANCY_DISTANCE + 1 ... MAX_OCCUPANCY_DISTANCE:
        occupancy = 100 - 5 * ((distance - MIN_OCCUPANCY_DISTANCE) / OCCUPANCY_DISTANCE_JUMP);
        break;
    default:
        occupancy = 0;
        break;
    }
    if (occupancy < 0)
        occupancy = 0;
    if (occupancy > 100)
        occupancy = 100;
}

// Kapı durumu kontrol ediliyor
void checkDoorState()
{
    digitalRead(DOOR_PIN) == 1 ? doorState = DOOR_CLOSED : doorState = DOOR_OPENED;
}

// Kapak durumu kontrol ediliyor
void checkCoverState()
{
    digitalRead(COVER_PIN) == 1 ? coverState = COVER_OPENED : coverState = COVER_CLOSED;
}

// Manyetik kilit durumu kontrol ediliyor
void checkMagneticLockState()
{
    digitalRead(MAGNETIC_LOCK_PIN) == 1 ? magneticLockState = MAGNETIC_UNLOCKED : magneticLockState = MAGNETIC_LOCKED;
}

// Kapı durumu alınıyor
String getDoorState()
{   
    return doorState == DOOR_CLOSED ? F("Kapali") : F("Acik");
}

// Kapak durumu alınıyor
String getCoverState()
{
    return coverState == COVER_OPENED ? F("Acik") : F("Kapali");
}

// Manyetik kilit durumu kontrol ediliyor
String getMagneticLockState()
{
    return magneticLockState == MAGNETIC_UNLOCKED ? F("Acik") : F("Kilitli");
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
    const int reading = analogRead(TEMPERATURE_PIN);
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    const float temperatureC = (voltage - 0.5) * 10;
    return temperatureC;
}

// Zaman fonksiyonu
String getLocalDateTime()
{
    String dateTime = "";
    dateTime += String(year());
    dateTime += F("/");
    dateTime += getDigits(month());
    dateTime += F("/");
    dateTime += getDigits(day());
    dateTime += F(" ");
    dateTime += getDigits(hour());
    dateTime += F(":");
    dateTime += getDigits(minute());
    dateTime += F(":");
    dateTime += getDigits(second());
       
    return dateTime;
}

// Sayıyı 2 basamaklı stringe çevirme fonksiyonu
String getDigits(const int digits)
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
    Serial.println(F("Resetting SIM808..."));
    digitalWrite(SIM808_POWER_PIN, HIGH);
    delay(3000);
    digitalWrite(SIM808_POWER_PIN, LOW);
    powerUpGPS();
}

// SIM808 GPS açma fonksiyonu
bool powerUpGPS()
{
    Serial.println(F("Powering up GPS..."));
    if (sim808_check_with_cmd("AT+CGPSPWR=1\r\n", "OK\r\n", CMD)) // GPS'i açma
    {
        Serial.println(F("GPS powered up!"));
        isGPSInitialized = true;
    }
    else
    {
        Serial.println(F("GPS power up failed!"));
        isGPSInitialized = false;
    }
    return isGPSInitialized;
}

// UTC formatındaki zamanı yerel saat formatına dönüştürme
String convertToLocalDate(const String time)
{
    const String year = time.substring(0, 4);
    const String month = time.substring(4, 6);
    const String day = time.substring(6, 8);
    const String hour = time.substring(8, 10);
    const String minute = time.substring(10, 12);
    const String second = time.substring(12, 14);
    if (!isSystemTimeSet && year.toInt() > 2022) {
        Serial.println(F("Setting system time..."));
        setTime(hour.toInt() + 3, minute.toInt(), second.toInt(), day.toInt(), month.toInt(), year.toInt());
        isSystemTimeSet = true;
    }
    const String localDate = day + F("/") + month + F("/") + year;
    const String localTime = hour + F(":") + minute + F(":") + second;
    return localDate + F(" ") + localTime;
}

// KPH formatındaki değeri KM/H formatına dönüştürme
float convertToKilometersPerHour(const String kphValue) 
{
    const float kph = kphValue.toFloat();
    const float kmh = kph * 1.852;
    return kmh;
}

// GPS verilerini ayrıştırma fonksiyonu
void parseGPSData(const String gpsData) 
{
    // İstenmeyen kısmı atlayarak veriyi parçalara bölme
    const int startIndex = gpsData.indexOf(F(":")) + 1;
    String data = gpsData.substring(startIndex);

    // Parçaları virgül karakterine göre ayırma
    int commaIndex = data.indexOf(F(","));
    const String mode = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String latitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String longitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String altitude = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String UTCTime = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String TTFF = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String num = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    commaIndex = data.indexOf(F(","));
    const String speed = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);

    const String course = data;

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
}

// DMM formatındaki değeri ondalık dereceye dönüştürme
float convertToDecimalDegrees(const String dmmValue) 
{
    const int degrees = dmmValue.substring(0, 2).toInt();
    const float minutes = dmmValue.substring(2).toFloat();
    const float decimalDegrees = degrees + (minutes / 60.0);
    return decimalDegrees;
}

void readGPS()
{
    if (!isGPSInitialized)
    {
        powerUpGPS();
    }
    const String gpsData = readSIM808("AT+CGPSINF=0\r\n", "CGPSINF: ");
    parseGPSData(gpsData);
}

String readSIM808(const String cmd, const String resp)
{
    String data = "";
    sim808_send_cmd(cmd.c_str());
    // int i = 0;
    do {
        data = mySerial.readStringUntil('\n');
        // Serial.print(F("Read: "));Serial.println(i++);
        // Serial.println(data);
        if (strstr(data.c_str(), resp.c_str()) != NULL)
        {
            // Serial.println(F("strstr(data, resp) != NULL"));
            return data;
        }
    } while (data != "");
    return data;
}

String getSIM808State()
{
    switch (sim808State)
    {
    case SIM808_POWER_DOWN:
        return F("POWER DOWN");
    case SIM808_POWER_UP:
        return F("POWER UP");
    case SIM808_SIM_NOT_READY:
        return F("SIM NOT");
    case SIM808_SIM_READY:
        return F("SIM READY");
    case SIM808_INITIALIZED:
        return F("SIM OK");
    case SIM808_JOINED_TO_NETWORK:
        return F("NET OK");
    case SIM808_TCP_CONNECTED:
        return F("TCP OK");
    case SIM808_REGISTERED_TO_SERVER:
        return F("SERVER OK");
    default:
        return F("CLOSE");
    }
}

void sendSystemStateToServer()
{
    if (sim808State == SIM808_POWER_DOWN)
    {
        if (sim808_check_with_cmd("AT\r\n", "OK\r\n", CMD))
        {
            sim808State = SIM808_POWER_UP;
            Serial.println(F("sim808.checkPowerUp() successfull"));
        } else {
            powerUpDownSIM808();
            Serial.println(F("sim808.checkPowerUp() failed"));
            sim808State = SIM808_POWER_DOWN;
        }
    }

    if (sim808State == SIM808_POWER_UP)
    {
        if (sim808.checkSIMStatus())
        {
            Serial.println(F("sim808.checkSIMStatus() successfull"));
            sim808State = SIM808_SIM_READY;
        } else {
            Serial.println(F("sim808.checkSIMStatus() failed"));
            sim808State = SIM808_SIM_NOT_READY;
        }
    }

    if (sim808State == SIM808_SIM_READY)
    {
        sim808.disconnect();
        if (sim808.init())
        {
            Serial.println(F("sim808.init() successfull"));
            sim808State = SIM808_INITIALIZED;
        } else {
            Serial.println(F("sim808.init() failed"));
            sim808State = SIM808_POWER_DOWN;
        }
    }

    if (sim808State == SIM808_INITIALIZED)
    {
        if (sim808.join(F("internet")))
        {
            Serial.println(F("SIM808_JOINED_TO_NETWORK"));
            sim808State = SIM808_JOINED_TO_NETWORK;
        } else {
            Serial.println(F("SIM808_NOT_JOINED_TO_NETWORK"));
            sim808State = SIM808_INITIALIZED;
        }
    }

    if (sim808State == SIM808_JOINED_TO_NETWORK)
    {
        if (sim808.connect(TCP, TCP_HOST, TCP_PORT))
        {
            Serial.println(F("SIM808_TCP_CONNECTED"));
            sim808State = SIM808_TCP_CONNECTED;
        } else {
            Serial.println(F("SIM808_TCP_NOT_CONNECTED"));
            sim808State = SIM808_INITIALIZED;
        }
    }

    if (sim808State == SIM808_TCP_CONNECTED)
    {
        const String imei = "IMEI:" + getIMEI();
        Serial.println(imei);
        const int ret = sim808.send(imei.c_str(), imei.length());
        if (ret > 0)
        {
            sim808State = SIM808_REGISTERED_TO_SERVER;
            Serial.println(F("SIM808_REGISTERED_TO_SERVER"));
            Serial.print(F("Sent: "));Serial.println(ret);
        } else {
            Serial.println(F("SIM808_SERVER_NOT_RESPONDING"));
            sim808State = SIM808_JOINED_TO_NETWORK;
        }
    }

    if (sim808State == SIM808_REGISTERED_TO_SERVER)
    {   
        const String systemState = getSystemState();
        Serial.println(F("Sending system state to server..."));
        sim808.send(systemState.c_str(), systemState.length());
        Serial.println(F("System state sent to server!"));

        // Serial.println(F("Waiting for server response..."));
        // const String response = recvFromServer();
        // Serial.print(F("Server Response: "));Serial.println(response);
    }
}

// Sistem durumunu döndüren fonksiyon
String getSystemState()
{
    String systemState = F("K:OK | ");
    systemState += F("Pres: ");
    systemState += checkPressState();
    systemState += F(" | ");
    systemState += F("Kapak: ");
    systemState += getCoverState();
    systemState += F(" | ");
    systemState += F("Kapı: ");
    systemState += getDoorState();
    systemState += F(" | ");
    systemState += F("Manyetik Kilit: ");
    systemState += getMagneticLockState();
    systemState += F(" | ");
    systemState += F("Doluluk: ");
    systemState += String(occupancy);
    systemState += F("% | ");
    systemState += F("Voltaj: ");
    systemState += String(voltage);
    systemState += F(" | ");
    systemState += F("Akım: ");
    systemState += String(current);
    systemState += F(" | ");
    systemState += F("Sıcaklık: ");
    systemState += temperature;
    systemState += F("C | ");
    systemState += F("Enlem: ");
    systemState += String(GPSdata.lat, 6);
    systemState += F(" | ");
    systemState += F("Boylam: ");
    systemState += String(GPSdata.lon, 6);
    systemState += F(" | ");
    systemState += F("Yerel Zaman: ");
    systemState += getLocalDateTime();

    return systemState;
}

void checkSIM808Status()
{
    if (sim808State >= SIM808_TCP_CONNECTED)
    {
        const String sim808Status = readSIM808("AT+CIPSTATUS\r\n", "STATE: ");
        Serial.println(F("SIM808 Status: "));
        Serial.println(sim808Status);

        if (strstr(sim808Status.c_str(), "CONNECT OK")){
            return;
        } else {
            Serial.println(F("SIM808 is not connected to server!"));
            sim808State = SIM808_POWER_DOWN;
        }
    }
}

String getIMEI()
{
    sim808_send_cmd("AT+GSN\r\n");
    mySerial.readStringUntil('\n');
    return mySerial.readStringUntil('\n');
}

void printToDisplayScreen(const int x, const int y, const String text)
{
    u8g2.setFont(u8g2_font_u8glib_4_tr);
    u8g2.setCursor(x, y);
    u8g2.println(text);
    u8g2.sendBuffer();
}

String recvFromServer() 
{
    const unsigned long startTime = millis();
    while (true) {
        const int ret = sim808.recv(buffer, sizeof(buffer)-1);
        if (ret <= 0){
            Serial.println(F("fetch over..."));
            break; 
        }
        buffer[ret] = '\0';
        Serial.print(F("Recv: "));
        Serial.print(ret);
        Serial.print(F(" bytes: "));
        Serial.println(buffer);
        break;
    }
    Serial.print(F("Response Time: "));Serial.println(millis() - startTime);
    return String(buffer);
}

String checkAvailableMessages() 
{   
    String message = "";
    while (mySerial.available())
    {
        message += mySerial.readStringUntil('\n');
        message += '\n';
    }
    return message;
}