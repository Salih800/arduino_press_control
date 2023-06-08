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

#define PRESS_CHECK_TIME 1000 // Pres kontrol zamanlayıcısı (Ne kadar sıklıkla presin yapılıp yapılmayacağı kontrol ediliyor)
#define PRESS_NEEDED_COUNT 60000 / PRESS_CHECK_TIME // Presin normal başlama sayısı (? milisaniye boyunca doluluk oranı %?'ın üzerindeyse)
#define MAX_PRESS_OFF_TIME 43200000UL // Presin maksimum durma süresi (Presin durduktan sonra ne kadar süre sonra tekrar çalışacağı)
#define SIM808_CHECK_TIME 1000 // GPS kontrol zamanlayıcısı
#define GPS_CHECK_TIME 30000 // GPS kontrol zamanlayıcısı
#define MESSAGE_SEND_INTERVAL 60000 // Mesaj gönderme aralığı
#define MESSAGE_SEND_TIMEOUT 10000 // Mesaj gönderme zaman aşımı
#define PRESS_WORKING_CURRENT 0.5 // Presin çalışma akımı
#define TEMPERATURE_CHECK_TIME 60000 // Sıcaklık kontrol zamanlayıcısı

#define NEEDED_OCCUPANCY 90 // Gerekli doluluk oranı
#define MIN_OCCUPANCY_DISTANCE 400 // Minimum doluluk mesafesi
#define MAX_OCCUPANCY_DISTANCE 1300 // Maksimum doluluk mesafesi
#define OCCUPANCY_DISTANCE_JUMP 45 // Doluluk mesafesi artışı
#define MIN_WORKING_VOLTAGE 22 // Minimum çalışma voltajı
#define MAX_WORKING_TEMPERATURE 70 // Maksimum çalışma sıcaklığı
#define SIM808_RESET_TIME 300000UL // SIM808 reset zamanı

#define SYSTEM_START_DELAY 1000 // Sistem başlatma gecikmesi

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

double voltage = 24.0;      // Voltaj değişkeni
double current = 0.0;      // Akım değişkeni
double power = 0.0;        // Güç değişkeni
double temperature = 0.0;  // Sıcaklık değişkeni

bool isPressReady = false;    // Pres hazır durumu
bool isPressWorking = false;  // Presin çalışma durumu
bool isPressPaused = true;   // Presin durdurulma durumu
bool isSystemTimeSet = false; // Sistemin zamanının ayarlanma durumu
bool isGPSInitialized = false; // GPS'in başlatılma durumu
bool isMessageSent = true; // Mesajın gönderilme durumu

String messagesToSend = ""; // Gönderilecek mesajlar
String receivedMessages = ""; // Alınan mesajlar
String IP = ""; // IP adresi

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
    SIM808_POWER_OFF,
    SIM808_POWER_ON,
    SIM808_GPS_POWERED,
    SIM808_GPS_READY,
    SIM808_SIM_NOT_READY,
    SIM808_SIM_READY,
    SIM808_INITIALIZED,
    SIM808_JOINED_TO_NETWORK,
    SIM808_WIRELESS_CONNECTED,
    SIM808_IP_CONNECTED,
    SIM808_TCP_CONNECTED,
    SIM808_REGISTERED_TO_SERVER,
};

// Pres durumları
enum PRESS_STATES {
    PRESS_DISABLED,
    PRESS_STOPPED,
    PRESS_NEEDED,
    PRESS_STARTING,
    PRESS_IS_GOING_DOWN,
    PRESS_DOWN_WAITING,
    PRESSED_DOWN,
    PRESS_IS_GOING_UP,
    PRESS_UP_WAITING,
    PRESSED_UP,
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
    OVERHEATED,
    UNDERVOLTAGE,
};

enum SIM808_STATES sim808State = SIM808_POWER_OFF; // SIM808 durumu
enum PRESS_STATES pressState = PRESS_STOPPED;       // Pres durumu
enum DOOR_STATES doorState = DOOR_CLOSED;         // Kapı durumu
enum COVER_STATES coverState = COVER_CLOSED;        // Kapak durumu
enum MAGNETIC_LOCK_STATES magneticLockState = MAGNETIC_UNLOCKED; // Manyetik kilit durumu
enum DISPLAY_STATES displayState = DOOR_AND_COVER_CLOSED; // Ekranda ne yazacağını belirleyen durumlar

// Zaman değişkenleri
unsigned long pressCheckTime = 0;            // Pres kontrol zamanlayıcısı
unsigned long sim808CheckTime = 0;              // SIM808 kontrol zamanlayıcısı
unsigned long gpsCheckTime = 0;              // GPS kontrol zamanlayıcısı
unsigned long displayCheckTime = 0;              // Ekran kontrol zamanlayıcısı
unsigned long lastPressTime = 0;             // Presin son çalışma zamanı
unsigned long lastMessageSendTime = 0;       // Son mesaj gönderme zamanı
unsigned long messageSentTime = 0;           // Mesajın gönderilme zamanı
unsigned long sim808ResetTime = 0;           // SIM808 reset zamanı
unsigned long systemLoopCounter = 0;          // Sistem döngü sayacı
double systemFPS = 0;                  // Sistem FPS'i
unsigned long temperatureCheckTime = 0;      // Sıcaklık kontrol zamanlayıcısı

// Fonksiyonlar
void setup(); // Setup fonksiyonu
void loop();  // Loop fonksiyonu
void updateVariables(); // Değişkenleri güncelleme fonksiyonu
void printVariablesToSerial(); // Değişkenleri seri porta yazdırma fonksiyonu
void checkPress(); // Pres kontrol fonksiyonu
void updatePressState(); // Presin durumunu güncelleme fonksiyonu
bool checkPressNeeded(); // Presin gerekli olup olmadığını kontrol eden fonksiyon
bool checkPressReady(); // Pres yapmaya hazır mı kontrol eden fonksiyon
String getPressState(); // Pres durumu kontrol ediliyor
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
// void powerUpDownSIM808(); // SIM808 güç açma kapama fonksiyonu
bool powerUpGPS(); // GPS güç açma fonksiyonu
String convertToLocalDate(const String time); // UTC zamanı yerel zamana çevirme fonksiyonu
float convertToKilometersPerHour(const String kphValue); // Hızı km/saat cinsinden çevirme fonksiyonu
void parseGPSData(const String gpsData); // GPS verilerini ayrıştırma fonksiyonu
float convertToDecimalDegrees(const String dmmValue); // DMM cinsinden koordinatı ondalık dereceye çevirme fonksiyonu
void readGPS(); // GPS verilerini okuma fonksiyonu
String readSIM808(const String cmd, const String resp); // SIM808 komutu okuma fonksiyonu
String getSIM808State(); // SIM808 durumunu döndüren fonksiyon
void sendSystemStateToServer(); // Sistem durumu sunucuya gönderiliyor
String getSystemState(); // Sistem durumunu döndüren fonksiyon
String getIMEI(); // IMEI numarasını döndüren fonksiyon
void printToDisplayScreen(const int x, const int y, const String text); // Ekrana yazı yazdırma fonksiyonu
void checkAvailableMessages(); // Sunucudan gelen verileri okuma fonksiyonu
void checkDisplayState(); // Ekranda ne yazacağını belirleyen fonksiyon
void updateTestDisplay(); // Test ekranını güncelleme fonksiyonu
void updateLiveDisplay(); // Canlı ekranı güncelleme fonksiyonu
void drawProgressBar(const int x, const int y, const int width, const int height, const int percent); // İlerleme çubuğu çizme fonksiyonu

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
    drawProgressBar(0, SCREEN_HEIGHT - 5, SCREEN_WIDTH, 5, 0);

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
    drawProgressBar(0, SCREEN_HEIGHT - 5, SCREEN_WIDTH, 5, 25);

    // Akım ve voltaj sensörü başlatılıyor
    Serial.println(F("Starting Power Module..."));
    printToDisplayScreen(0, 25, F("Guc Modulu:"));
    if (powerSensor.begin()) // TODO: Check if this is working
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
    drawProgressBar(0, SCREEN_HEIGHT - 5, SCREEN_WIDTH, 5, 50);
    
    // Sim808 başlatılıyor
    Serial.println(F("Starting SIM808..."));
    printToDisplayScreen(0, 35, F("SIM808:"));
    sim808_send_cmd("AT+CPIN?\r\n");
    delay(1000);
    if (mySerial.available())
    {
        Serial.println(F("SIM808 Started."));
        printToDisplayScreen(64, 35, F("Baslatildi."));
        sim808State = SIM808_POWER_ON;
    } else {
        Serial.println(F("SIM808 Failed!"));
        printToDisplayScreen(64, 35, F("Baslatilamadi!"));
        sim808State = SIM808_POWER_OFF;
    }
    drawProgressBar(0, SCREEN_HEIGHT - 5, SCREEN_WIDTH, 5, 75);

    // SIM808 GPS başlatılıyor
    Serial.println(F("Starting GPS..."));
    printToDisplayScreen(0, 45, F("GPS:"));
    if (powerUpGPS()) // GPS'i açma
    {
        Serial.println(F("GPS Started."));
        printToDisplayScreen(64, 45, F("Baslatildi."));
        sim808State = SIM808_POWER_ON;
    } else {
        Serial.println(F("GPS Failed!"));
        printToDisplayScreen(64, 45, F("Baslatilamadi!"));
        sim808State = SIM808_POWER_OFF;
    }
    drawProgressBar(0, SCREEN_HEIGHT - 5, SCREEN_WIDTH, 5, 100);

    printToDisplayScreen(0, 55, F("Sistem Baslatildi."));
    Serial.println(F("System Started."));
    delay(SYSTEM_START_DELAY);
}

void loop()
{
    systemLoopCounter++; // Sistem FPS sayacı arttırılıyor
    systemFPS = (float)systemLoopCounter / (float)(millis()) * 1000.0; // Sistem FPS hesaplanıyor
    updateVariables();        // Değişkenler güncelleniyor
    // printVariablesToSerial(); // Değişkenler seri porta yazdırılıyor

    // Pres kontrol ediliyor
    checkPress();

    // Gelen veriler okunuyor
    checkAvailableMessages(); // Sunucudan gelen veriler okunuyor
    
    // GPS kontrol ediliyor
    if (millis() - gpsCheckTime > GPS_CHECK_TIME && !isPressWorking && isMessageSent)
    {
        gpsCheckTime = millis();
        readGPS();
        // Serial.print(F("GPS check time: "));Serial.println(millis() - gpsCheckTime);
    }

    // SIM808 kontrol ediliyor
    if (millis() - sim808CheckTime > SIM808_CHECK_TIME) // && !isPressWorking)
    {
        sim808CheckTime = millis();
        sendSystemStateToServer();
        // Serial.print(F("SIM808 check time: "));Serial.println(millis() - sim808CheckTime);
    }

    // Ekran güncelleniyor
    // updateLiveDisplay();
    updateTestDisplay();
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
    if (millis() - temperatureCheckTime > TEMPERATURE_CHECK_TIME || temperatureCheckTime == 0)
    {
        temperatureCheckTime = millis();
        temperature = readTemperature();
    }
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
    Serial.print(F("Press: "));Serial.print(getPressState());Serial.print(F(" | "));
    Serial.print(F("SIM: "));Serial.print(getSIM808State());
    Serial.println();
}

// Pres kontrol fonksiyonu
void checkPress()
{
    checkPressWorking(); // Presin çalışıp çalışmadığı kontrol ediliyor
    checkPressReady();   // Presin hazır olup olmadığı kontrol ediliyor

    if (millis() - pressCheckTime > PRESS_CHECK_TIME && pressState <= PRESS_STOPPED)
    {
        pressCheckTime = millis(); // TODO: PRESS NEEDED
        if (checkPressNeeded())
        {
            pressNeeded++;
        }
        else
        {
            pressState = PRESS_STOPPED;
            pressNeeded = 0;
        }
    }

    if (pressState == PRESS_STOPPED)
    {
        if (millis() - lastPressTime > MAX_PRESS_OFF_TIME || pressNeeded > PRESS_NEEDED_COUNT)
        {
            pressState = PRESS_NEEDED;
        }
    }
    
    updatePressState(); // Pres durumu güncelleniyor
}

// Presin durumunu güncelleme fonksiyonu
void updatePressState() 
{
    if (pressDownState == 0 && pressUpState == 0 && !isPressWorking && isPressReady && pressState == PRESS_NEEDED)
    { // Pres duruyorsa ve pres hazırsa ve pres gerekliyse ve pres çalışmıyorsa ve pres devre dışı değilse
        // Serial.println(F("Starting press..."));
        pressDown();
        pressState = PRESS_STARTING;
    }
    else if (pressDownState == 1 && pressUpState == 0 && isPressWorking && isPressReady)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışıyorsa
        // Serial.println(F("Pressing down..."));
        pressDown();
        pressState = PRESS_IS_GOING_DOWN;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressReady)
    { // Pres aşağı iniyorsa ve pres hazır değilse
        // Serial.println(F("Waiting for pressing down..."));
        pressWait();
        pressState = PRESS_DOWN_WAITING;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        // Serial.println(F("Continuing pressing down..."));
        pressDown();
        pressState = PRESS_IS_GOING_DOWN;
    }
    else if (pressDownState == 1 && pressUpState == 0 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres aşağı iniyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        // Serial.println(F("Pressed down..."));
        pressUp();
        pressState = PRESSED_DOWN;
    }
    else if (pressDownState == 0 && pressUpState == 1 && isPressWorking && isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışıyorsa
        // Serial.println(F("Pressing up..."));
        pressUp();
        pressState = PRESS_IS_GOING_UP;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressReady)
    { // Pres yukarı çıkıyorsa ve pres hazır değilse
        // Serial.println(F("Waiting for pressing up..."));
        pressWait();
        pressState = PRESS_UP_WAITING;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmuşsa
        // Serial.println(F("Continuing pressing up..."));
        pressUp();
        pressState = PRESS_IS_GOING_UP;
    }
    else if (pressDownState == 0 && pressUpState == 1 && !isPressWorking && isPressReady && !isPressPaused)
    { // Pres yukarı çıkıyorsa ve pres hazırsa ve pres çalışmıyorsa ve pres durdurulmamışsa
        // Serial.println(F("Pressed up..."));
        pressState = PRESSED_UP;
        pressStop();
        pressNeeded = 0;
        lastPressTime = millis();
        checkPressNeeded() ? pressState = PRESS_DISABLED : pressState = PRESS_STOPPED;
    }
}

// Presin gerekli olup olmadığını kontrol eden fonksiyon
bool checkPressNeeded()
{
    return occupancy >= NEEDED_OCCUPANCY;
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

String getPressState()
{
    switch (pressState)
    {
    case PRESS_DISABLED:
        return F("Devre disi");
    case PRESS_STOPPED:
        return F("Duruyor");
    case PRESS_NEEDED:
        return F("Gerekli");
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

// Manyetik kilit durumu alınıyor
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
// void powerUpDownSIM808()
// {
//     Serial.println(F("Resetting SIM808..."));
//     digitalWrite(SIM808_POWER_PIN, HIGH);
//     delay(3000);
//     digitalWrite(SIM808_POWER_PIN, LOW);
//     delay(1000);
//     powerUpGPS();
// }

// SIM808 GPS açma fonksiyonu
bool powerUpGPS()
{
    Serial.println(F("Powering up GPS..."));
    if (sim808_check_with_cmd("AT+CGPSPWR=1\r\n", "OK\r\n", CMD) && sim808_check_with_cmd("AT+CGNSTST=0\r\n", "OK\r\n", CMD)) // GPS'i açma
    {   
        Serial.println(F("GPS powered up!"));
        isGPSInitialized = true; // TODO: GPS'in açık olup olmadığını kontrol et
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
        Serial.println(F("Updating system time..."));
        setTime(hour.toInt(), minute.toInt(), second.toInt(), day.toInt(), month.toInt(), year.toInt());
        adjustTime(3 * SECS_PER_HOUR); // UTC+3
        isSystemTimeSet = true;
        Serial.println(F("System time updated!"));
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
    Serial.print(F("GPS Data: "));Serial.println(gpsData);
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
    if (UTCTime.length() == 18){
        GPSdata.localTime = convertToLocalDate(UTCTime);
    }
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
    // sim808_send_cmd("AT+CGPSINF=0\r\n");
    const String gpsData = readSIM808("AT+CGPSINF=0\r\n", "CGPSINF: ");
    if (gpsData == "")
    {
        Serial.println(F("GPS data is empty!"));
        return;
    } else if (gpsData.length() < 70)
    {
        Serial.println(F("GPS data is invalid!"));
    } else
    {
        parseGPSData(gpsData);
        // Serial.println(F("GPS data is valid!"));
    }
    // mySerial.readStringUntil('\n');
    // mySerial.readStringUntil('\n');
}

String readSIM808(const String cmd, const String resp)
{
    String data = "";
    sim808_send_cmd(cmd.c_str());
    do {
        data = mySerial.readStringUntil('\n');
        if (strstr(data.c_str(), resp.c_str()) != NULL)
        {
            return data;
        } else {
            receivedMessages += data;
        }
    } while (data != "");
    return data;
}

// SIM808 durumunu döndüren fonksiyon
String getSIM808State()
{
    switch (sim808State)
    {
    case SIM808_POWER_OFF:
        return F("POWER OFF");
    case SIM808_POWER_ON:
        return F("POWER ON");
    case SIM808_GPS_POWERED:
        return F("GPS OK");
    case SIM808_GPS_READY:
        return F("GPS READY");
    case SIM808_SIM_NOT_READY:
        return F("NO SIM");
    case SIM808_SIM_READY:
        return F("SIM READY");
    case SIM808_INITIALIZED:
        return F("READY");
    case SIM808_JOINED_TO_NETWORK:
        return F("NET OK");
    case SIM808_WIRELESS_CONNECTED ... SIM808_IP_CONNECTED:
        return F("CONNECTING");
    case SIM808_TCP_CONNECTED:
        return F("TCP OK");
    case SIM808_REGISTERED_TO_SERVER:
        return F("SERVER OK");
    default:
        return F("UNKNOWN");
    }
}

// Sunucuya bağlanma ve sistem durumunu gönderme fonksiyonu
void sendSystemStateToServer()
{
    if (sim808State == SIM808_POWER_OFF)
    {
        sim808_send_cmd("AT+CFUN=1\r\n");
    }

    if (sim808State == SIM808_POWER_ON)
    {
        sim808_send_cmd("AT+CGPSPWR=1\r\n");
    }

    if (sim808State == SIM808_GPS_POWERED) {
        sim808_send_cmd("AT+CGNSTST=0\r\n");
    }
    
    if (sim808State == SIM808_GPS_READY || sim808State == SIM808_SIM_NOT_READY) 
    {
        sim808_send_cmd("AT+CPIN?\r\n");
    }

    if (sim808State == SIM808_SIM_READY)
    {
        Serial.println(F("Closing previous connection..."));
        sim808_send_cmd("AT+CIPSHUT\r\n");
        messagesToSend = "";
        lastMessageSendTime = 0;
        isMessageSent = true;
    }

    if (sim808State == SIM808_INITIALIZED)
    {
        sim808_send_cmd("AT+CSTT=\"internet\",\"\",\"\"\r\n");
    }

    if (sim808State == SIM808_JOINED_TO_NETWORK)
    {
        sim808_send_cmd("AT+CIICR\r\n");
    }

    if (sim808State == SIM808_WIRELESS_CONNECTED)
    {
        sim808_send_cmd("AT+CIFSR\r\n");
    }

    if (sim808State == SIM808_IP_CONNECTED)
    {
        const String reg_cmd = "AT+CIPSTART=\"TCP\",\"" + String(TCP_HOST) + "\"," + String(TCP_PORT) + "\r\n";
        sim808_send_cmd(reg_cmd.c_str());
    }

    if (sim808State == SIM808_TCP_CONNECTED)
    {
        const String imei = "IMEI:" + getIMEI();
        messagesToSend += imei + "\n";
    }

    if (sim808State == SIM808_REGISTERED_TO_SERVER)
    {   
        if (millis() - lastMessageSendTime > MESSAGE_SEND_INTERVAL || lastMessageSendTime == 0) {
            lastMessageSendTime = millis();
            messagesToSend += getSystemState() + ("\n");
        }
    }

    if (sim808State < SIM808_REGISTERED_TO_SERVER) {
        if (millis() - sim808ResetTime > SIM808_RESET_TIME) {
            Serial.println(F("Resetting SIM808..."));
            // mySerial.flush();
            delay(100);
            sim808_send_cmd("AT+CFUN=1,1\r\n");
            delay(100);
            sim808State = SIM808_POWER_OFF;
            sim808ResetTime = millis();
        }
    } else {
        sim808ResetTime = millis();
    }

    if (messagesToSend.length() > 0 && sim808State >= SIM808_TCP_CONNECTED) {
        const char *messageToSend = strtok((char *)messagesToSend.c_str(), "\n");
        const String message = String(messageToSend);
        if (message && isMessageSent){ // && !isPressWorking) {
            isMessageSent = false;
            messagesToSend = messagesToSend.substring(message.length() + 1);
            // Serial.print(F("Message to send: "));Serial.println(message);
            // Serial.print(message.length());Serial.println(F(" bytes"));
            if (sim808.send(message.c_str(), message.length()) == 0) {
                sim808State = SIM808_SIM_READY;
                Serial.println(F("Message could not be sent!"));
            }
            messageSentTime = millis();
        }
    }
}

// Sistem durumunu döndüren fonksiyon
String getSystemState()
{
    String systemState = F("K:OK | ");
    systemState += F("Pres: ");
    systemState += getPressState();
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

// IMEI numarasını döndüren fonksiyon
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

void checkAvailableMessages() 
{
    // unsigned long messageCheckTime = millis();
    while (mySerial.available()) {
        receivedMessages += mySerial.readStringUntil('\n') + F("\n");
    } 
    char *message = strtok((char *)receivedMessages.c_str(), "\n");

    while (message)
    {
        Serial.print(F("New Message: "));Serial.println(message);
        if (strstr(message, "+PDP: DEACT") != NULL || strstr(message, "CLOSED") != NULL || strstr(message, "+CPIN: NOT READY") != NULL) {
            sim808State = SIM808_POWER_OFF;
            Serial.println(F("SIM808_POWER_OFF"));
        }

        if (strstr(message, "SEND OK") != NULL) {
            Serial.println(F("Message sent to server!"));
            isMessageSent = true;
        }

        switch (sim808State){
            case SIM808_POWER_OFF:
                if (strstr(message, "OK") != NULL) {
                    sim808State = SIM808_POWER_ON;
                    Serial.println(F("SIM808_POWER_ON"));
                }
                break;
            case SIM808_POWER_ON:
                if (strstr(message, "OK") != NULL) {
                    sim808State = SIM808_GPS_POWERED;
                    Serial.println(F("SIM808_SIM_NOT_READY"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_POWER_OFF;
                    Serial.println(F("SIM808_POWER_OFF"));
                }
                break;
            case SIM808_GPS_POWERED:
                if (strstr(message, "OK") != NULL) {
                    sim808State = SIM808_GPS_READY;
                    Serial.println(F("SIM808_SIM_NOT_READY"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_POWER_ON;
                    Serial.println(F("SIM808_POWER_ON"));
                }
                break;
            case SIM808_GPS_READY ... SIM808_SIM_NOT_READY:
                if (strstr(message, "+CPIN: READY") != NULL) {
                    sim808State = SIM808_SIM_READY;
                    Serial.println(F("SIM808_SIM_READY"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_SIM_NOT_READY;
                    Serial.println(F("SIM808_SIM_NOT_READY"));
                }
                break;
            case SIM808_SIM_READY:
                if (strstr(message, "SHUT OK") != NULL) {
                    sim808State = SIM808_INITIALIZED;
                    Serial.println(F("SIM808_INITIALIZED"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_SIM_NOT_READY;
                    Serial.println(F("SIM808_SIM_NOT_READY"));
                }
                break;
            case SIM808_INITIALIZED:
                if (strstr(message, "OK") != NULL) {
                    sim808State = SIM808_JOINED_TO_NETWORK;
                    Serial.println(F("SIM808_JOINED_TO_NETWORK"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_SIM_READY;
                    Serial.println(F("SIM808_CANNOT_JOIN_TO_NETWORK"));
                }
                break;
            case SIM808_JOINED_TO_NETWORK:
                if (strstr(message, "OK") != NULL) {
                    sim808State = SIM808_WIRELESS_CONNECTED;
                    Serial.println(F("SIM808_WIRELESS_CONNECTED"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_INITIALIZED;
                    Serial.println(F("SIM808_CANNOT_BRING_UP_WIRELESS_CONNECTION"));
                }
                break;
            case SIM808_WIRELESS_CONNECTED:
                if (strstr(message, ".") != NULL) {
                    sim808State = SIM808_IP_CONNECTED;
                    IP = message;
                    Serial.println(F("SIM808_IP_CONNECTED"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_JOINED_TO_NETWORK;
                    Serial.println(F("SIM808_CANNOT_GET_IP"));
                }
                break;
            case SIM808_IP_CONNECTED:
                if (strstr(message, "CONNECT OK") != NULL) {
                    sim808State = SIM808_TCP_CONNECTED;
                    Serial.println(F("SIM808_TCP_CONNECTED"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_WIRELESS_CONNECTED;
                    Serial.println(F("SIM808_CANNOT_CONNECT_TO_SERVER"));
                }
                break;
            case SIM808_TCP_CONNECTED:
                if (strstr(message, "OK:1") != NULL) {
                    sim808State = SIM808_REGISTERED_TO_SERVER;
                    Serial.println(F("SIM808_REGISTERED_TO_SERVER"));
                } else if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_IP_CONNECTED;
                    Serial.println(F("SIM808_TCP_NOT_CONNECTED"));
                }
                break;
            case SIM808_REGISTERED_TO_SERVER:
                if (strstr(message, "ERROR") != NULL) {
                    sim808State = SIM808_TCP_CONNECTED;
                    Serial.println(F("SIM808_TCP_CONNECTION_LOST"));
                } else if (strstr(message, "K:PRES DURUMU") != NULL) {
                    messagesToSend += "PRES DURUMU:" + getPressState() + F("\n");
                } else if (strstr(message, "K:PRESİ BAŞLAT") != NULL) {
                    pressDownState = 1;
                    pressUpState = 0;
                    pressState = PRESS_STARTING;
                    isPressPaused = true;
                    Serial.println(F("Press started!"));
                    messagesToSend += "PRESİ BAŞLAT: OK\n";
                } else if (strstr(message, "K:PRESİ DURDUR") != NULL) {
                    pressDownState = 0;
                    pressUpState = 1;
                    isPressPaused = true;
                    messagesToSend += F("PRESİ DURDUR: OK\n");
                } else if (strstr(message, "K:KONUM") != NULL) {
                    const String location = String(GPSdata.lat, 6) + F(",") + String(GPSdata.lon, 6);
                    messagesToSend += "KONUM: " + location + F("\n");
                } else if (strstr(message, "K:SICAKLIK") != NULL) {
                    messagesToSend += "SICAKLIK: " + String(temperature) + F("C\n");
                } else if (strstr(message, "K:VOLTAJ") != NULL) {
                    messagesToSend += "VOLTAJ: " + String(voltage) + F("V\n");
                } else if (strstr(message, "K:DOLULUK") != NULL) {
                    messagesToSend += ("DOLULUK: ") + String(occupancy) + F("%\n");
                } else if (strstr(message, "K:IP") != NULL) {
                    messagesToSend += "IP: " + IP + F("\n");
                }
                break;
            default:
                Serial.println(F("SIM808_UNKNOWN_STATE"));
                break;
        }

        message = strtok(NULL, "\n");
    }

    receivedMessages = "";

    if (!isMessageSent && millis() - messageSentTime > MESSAGE_SEND_TIMEOUT) {
        Serial.println(F("Message send timeout! Closing connection..."));
        sim808State = SIM808_SIM_READY;
    }

    // messageCheckTime = millis() - messageCheckTime;
    // if (messageCheckTime > 5) {
    //     Serial.print(F("Message Check Time: "));Serial.println(messageCheckTime);
    // }
}

void checkDisplayState()
{
    if (voltage < MIN_WORKING_VOLTAGE) {
        displayState = UNDERVOLTAGE;
    } else if (temperature > MAX_WORKING_TEMPERATURE) {
        displayState = OVERHEATED;
    } else if (doorState == DOOR_CLOSED && coverState == COVER_CLOSED && !isPressWorking) {
        displayState = DOOR_AND_COVER_CLOSED;
    } else if (doorState == DOOR_CLOSED && coverState == COVER_CLOSED && isPressWorking) {
        displayState = PRESS_WORKING;
    } else if (doorState == DOOR_OPENED && coverState == COVER_CLOSED) {
        displayState = DOOR_OPENED_COVER_CLOSED;
    } else if (doorState == DOOR_CLOSED && coverState == COVER_OPENED) {
        displayState = DOOR_CLOSED_COVER_OPENED;
    } else if (doorState == DOOR_OPENED && coverState == COVER_OPENED) {
        displayState = DOOR_AND_COVER_OPENED;
    }
}

void updateTestDisplay()
{
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
        u8g2.setCursor(0, 62);u8g2.print(F("FPS: "));u8g2.println(systemFPS);

        u8g2.setCursor(64, 5);u8g2.print(F("Doluluk: "));u8g2.print(occupancy);u8g2.println(F(" %"));
        u8g2.setCursor(64, 15);u8g2.print(F("Kapak: "));u8g2.println(getCoverState());
        u8g2.setCursor(64, 25);u8g2.print(F("Kapi: "));u8g2.println(getDoorState());
        u8g2.setCursor(75, 35);u8g2.print(F("M.Kilit: "));u8g2.println(getMagneticLockState());
        u8g2.setCursor(64, 45);u8g2.print(F("Pres: "));u8g2.println(getPressState());
        u8g2.setCursor(64, 55);u8g2.print(GPSdata.lat, 4);u8g2.print(F(","));u8g2.print(GPSdata.lon, 4);
        u8g2.setCursor(64, 62);u8g2.print(F("mainv4_live"));

    } while (u8g2.nextPage());
}

// Ekranı güncelleme fonksiyonu
void updateLiveDisplay()
{
    checkDisplayState();
    String displayMessage = "";
    switch (displayState)
    {
    case UNDERVOLTAGE:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(30,30,"DUSUK");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(27,50,"VOLTAJ");
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(46,13,". .");
        u8g2.drawStr(63,35,".");
        u8g2.drawStr(75,13,". .");
        u8g2.sendBuffer();
        break;
    case OVERHEATED:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(25,30,"YUKSEK");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(17,50,"SICAKLIK");
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(40,13,". .");
        u8g2.sendBuffer();
        break;
    case DOOR_AND_COVER_CLOSED:
        if (millis() - displayCheckTime < 10000)
        {
            u8g2.clearBuffer(); 
            u8g2.setFont(u8g2_font_logisoso34_tf);  
            u8g2.drawStr(5,41,"PENDIK"); 
            u8g2.setFont(u8g2_font_logisoso34_tf); 
            u8g2.drawStr(90,5,".");
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(2,64,"BELEDIYESI");
            u8g2.setFont(u8g2_font_logisoso34_tf); 
            u8g2.drawStr(67,48,".");
            u8g2.drawStr(116,48,".");
            u8g2.sendBuffer();
        } else if (millis() - displayCheckTime < 20000) {
            displayMessage = getLocalDateTime();
            u8g2.clearBuffer();
            // u8g2.setFont(u8g2_font_6x13B_tr); //u8g2_font_t0_11_tr
            u8g2.setFont(u8g2_font_6x13B_tr);
            u8g2.drawStr(7, 20,displayMessage.c_str());
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(5, 40, "DOLULUK");
            u8g2.setFont(u8g2_font_ncenB12_tr);
            u8g2.drawStr(15, 60, "%");
            u8g2.drawStr(30, 60, String(occupancy).c_str());

            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(69,40,"SICAKLIK ");
            u8g2.setFont(u8g2_font_ncenB12_tr);
            u8g2.drawStr(75, 60, String((int)temperature).c_str());
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(95,53,"o");
            u8g2.setFont(u8g2_font_ncenB12_tr);
            u8g2.drawStr(102,60,"C");  
            
            u8g2.sendBuffer();
        } else {
            displayCheckTime = millis();
        }
        break;
    case PRESS_WORKING:
        u8g2.clearBuffer();    
        u8g2.setFont(u8g2_font_ncenB18_tr); 
        u8g2.drawStr(25,30,"PRES");
        u8g2.setFont(u8g2_font_ncenB14_tr); 
        u8g2.drawStr(5,55,"YAPILIYOR");
        u8g2.sendBuffer();
        break;
    case DOOR_OPENED_COVER_CLOSED:
        u8g2.clearBuffer();                
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(40,30,"KAPI");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(30,50,"ACILDI");
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(48,55,".");
        u8g2.sendBuffer(); 
        break;
    case DOOR_CLOSED_COVER_OPENED:
        u8g2.clearBuffer();                
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(30,30,"KAPAK");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(30,50,"ACILDI");
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(48,55,".");
        u8g2.sendBuffer(); 
        break;
    case DOOR_AND_COVER_OPENED:
        u8g2.clearBuffer();                
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.drawStr(3,30,"KAPAK VE KAPI");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(30,50,"ACILDI");
        u8g2.setFont(u8g2_font_ncenB12_tr);
        u8g2.drawStr(48,55,".");
        u8g2.sendBuffer();
        break;
    default:
        break;
    }
}

void drawProgressBar(const int x, const int y, const int width, const int height, const int progress)
{
    u8g2.drawFrame(x, y, width, height);
    u8g2.drawBox(x, y, width * progress / 100, height);
    u8g2.sendBuffer();
}