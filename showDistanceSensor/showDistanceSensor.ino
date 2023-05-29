// #include <U8g2lib.h>  // Ekran için
// #include <VL53L0X.h>  // Uzaklık sensörü için
// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif
// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif

// VL53L0X distanceSensor;
// U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ

// int distanceInMM = 0;

// void setup() {
//     Serial.begin(9600);
//     Serial.println("Initializing Distance Sensor...");

//     u8g2.begin();
//     u8g2.clearBuffer();
//     u8g2.setFont(u8g2_font_ncenB08_tr);
//     u8g2.clearBuffer();
//     u8g2.drawStr(0, 10, "Distance Sensor");
//     u8g2.drawStr(0, 20, "Initializing...");
//     u8g2.sendBuffer();
//     delay(1000);
//     Wire.begin(2);
//     distanceSensor.init();
//     distanceSensor.setTimeout(500);
//     distanceSensor.startContinuous();

//     delay(1000);

//     Serial.println("Distance Sensor Initialized");
// }

// void loop() {
//     if (!distanceSensor.init()) {
//         Serial.println("Failed to detect Distance Sensor");
//         u8g2.clearBuffer();
//         u8g2.drawStr(0, 10, "Distance Sensor");
//         u8g2.drawStr(0, 20, "Failed to detect");
//         u8g2.drawStr(0, 30, "sensor!");
//         u8g2.sendBuffer();
//         return;
//     }
//     if (distanceSensor.timeoutOccurred()) {
//         Serial.println("Timeout to detect Distance");
//         u8g2.clearBuffer();
//         u8g2.drawStr(0, 10, "Distance Sensor");
//         u8g2.drawStr(0, 20, "Timeout!");
//         u8g2.sendBuffer();
//         return;
//     }
//     distanceInMM = distanceSensor.readRangeContinuousMillimeters();
//     u8g2.clearBuffer();
//     u8g2.drawStr(0, 10, "Distance Sensor");
//     u8g2.drawStr(0, 20, "Distance in MM: ");
//     u8g2.setCursor(60, 20);
//     u8g2.print(distanceInMM);
//     u8g2.drawStr(0, 40, "Distance in CM: ");
//     u8g2.setCursor(60, 40);
//     u8g2.print(distanceInMM / 10);
//     u8g2.sendBuffer();
// }

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <U8g2lib.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12);

int distanceInMM = 0;
int doluluk = 0;

void setup()
{
    pinMode(7, OUTPUT);
    delay(4000);
    digitalWrite(7, HIGH);
    delay(1000);
    Serial.begin(9600);
    u8g2.begin(); // EKRAN KODU
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(5, 30, "YUKLENIYOR...");
    u8g2.sendBuffer();
    Wire.begin(2);
    lox.begin();
}

void loop()
{
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    distanceInMM = measure.RangeMilliMeter;
    Serial.print("Mesafe: ");
    Serial.println(distanceInMM);

    if (distanceInMM <= 300)
    {
        doluluk = 100;
    }
    else if (distanceInMM <= 1200)
    {
        doluluk = map(distanceInMM, 300, 1200, 100, 0);
    }
    else
    {
        doluluk = 0;
    }

    if (measure.RangeStatus != 4) // Hedef algılandıysa
    {
        int mesafe_cm = measure.RangeMilliMeter / 10; // Veriyi mm'den cm'ye dönüştürün
        u8g2.firstPage();
        do
        {
            u8g2.setCursor(0, 20);
            u8g2.print("Mesafe: ");
            u8g2.print(mesafe_cm);
            u8g2.print(" cm");
            u8g2.setCursor(0, 40);
            u8g2.print("Doluluk: ");
            u8g2.print(doluluk);
            Serial.print("Mesafe: ");
            Serial.println(mesafe_cm);
            Serial.print("Doluluk: ");
            Serial.println(doluluk);
        } while (u8g2.nextPage());
    }
    else
    {
        u8g2.firstPage();
        do
        {
            u8g2.setCursor(0, 20);
            u8g2.print("Hedef algilanamadi");
            Serial.println("Hedef algilanamadi");
        } while (u8g2.nextPage());
    }

    delay(100); // 100ms bekle
}
