// KÜTÜPHANELER
#include <DFRobot_SIM808.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include "INA226.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
// KÜTÜPHANELER

SoftwareSerial myGsm(10, 11);
DFRobot_SIM808 sim808(&myGsm); // Connect RX,TX,PWR,

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/13, /* data=*/3, /* CS=*/2, /* reset=*/12); // EKRAN MODELİ

#include <VL53L0X.h> // MESAFE SENSÖRÜ
VL53L0X sensor;      // MESAFE SENSÖRÜ
INA226 INA(0x40);    // GÜÇ ANALİZÖRÜ

String latitude, longitude, tarih, enlem, boylam, yil, gun, ay, saat, dakika, rakim, hiz, zaman; // KONUM İÇİN EKLENDİ
String konum;                                                                                    // KONUM İÇİN EKLENDİ

#define bir 1      // KONUM İ ÇİN EKLENDİ
#define onn 100    // KONUM İÇİN EKLENDİ
#define yuz 100    // KONUM İÇİN EKLENDİ
#define ikiyuz 500 // KONUM İÇİN EKLENDİ

int k = 8; // MOTOR İLERİ
int v = 9; // MOTOR GERİ
int i = 0;
int msfinaktif = 0;
int ekransayaci = 0;
long lcdsayaci = 0;  // ekran sayacı
long ihlalterm = 0;  // sicaklık ihlal sayacı
long ihlalkapi = 0;  // kapı ihlal sayacı
long ihlalkapak = 0; // kapak ihlal sayacı
long sayacbir = 0;   // pres sayacı
long sayaciki = 0;   // 2 saatlik sayacı
long sayacuc = 0;    // i2c sensörlerini 2 sn de bir okur
long sayacdort = 0;  // serial den yazdırma 2 sn de bir
long sayacbes = 0;   // sicaklik sayaci
int manyet = 6;      // manyet
long sayacmsf = 0;

float voltaj = 25.4;
float akim = 0;

int c = 0; // mesafe sensörü
int doluluk = 0;
int ortdoluluk = 0;
int doluluksayac = 0;
int presdoluluk = 0;
int presikileme = 0;

int maxterm = 70; // SICAKLIK SENSÖRÜ
int termPin = A0; // SICAKLIK SENSÖRÜ
int sicaklik = 0; // SICAKLIK SENSÖRÜ
int s = 0;
int stp = 0;

bool kapakdurum = 0; // KAPAK SENSÖRÜ KODU - KAPAK DURUMU 1 YANİ HIGH İSE KAPAK AÇIKTIR.
bool kapidurum = 0;  // KAPI SENSÖRÜ KODU  - KAPI DURUMU 0 İSE YANİ LOW İSE KAPI AÇIKTIR.
int kapi = 5;
int kapak = 4;

unsigned long donguzaman = 0;  // KONUM İÇİN EKLENDİ
unsigned long serialzaman = 0; // KONUM İÇİN EKLENDİ
String msggelen;               // KONUM İÇİN EKLENDİ
String msggiden;               // KONUM İÇİN EKLENDİ
long konumsayac = 0;

String temp;
float temp_2;
long simsayac = 0;

String getValue(String data, char separator, int index) // KONUM İÇİN EKLENDİ
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void gpskonum() // KONUM İÇİN EKLENDİ
{
  Serial.println("GPS Kontrol Ediliyor");
  myGsm.print("AT+CGNSINF\r\n");
  delay(bir);
  readsimPort();

  if (msggelen != "")
  {
    Serial.println("Konum sim808 GELEN:  ");
    Serial.println(msggelen);
  }

  if ((msggelen.indexOf("+CGNSINF: 1,1,") > -1))
  {
    Serial.println("GPS Aktif Konum Bulundu :");
    delay(onn);
    tarih = getValue(msggelen, ',', 2);
    delay(onn);
    yil = tarih.substring(0, 4);
    delay(onn);
    ay = tarih.substring(4, 6);
    delay(onn);
    gun = tarih.substring(6, 8);
    delay(onn);
    enlem = getValue(msggelen, ',', 3);
    delay(onn);
    latitude = enlem.substring(0, 10);
    Serial.println(latitude);
    delay(onn);
    boylam = getValue(msggelen, ',', 4);
    delay(onn);
    longitude = boylam.substring(0, 11);
    Serial.println(longitude);
    delay(onn);
    msggelen = "";
    delay(onn);
    readsimPort();
    delay(onn);

    delay(50);
    delay(onn);

    konumsayac = konumsayac + 10;

    if (konumsayac == 10)
    {
      konum = ("https://www.google.com/maps/search/?api=1&query=" + latitude + "," + longitude);
    }
    //    delay(1000);

    if (konumsayac == 100)
    {
      myGsm.println("AT+CIPSEND");
    }

    //  delay(2000);

    if (konumsayac == 300)
    {
      printSerialData();
      myGsm.print("Konum: ");
      myGsm.println(konum);
    }

    //   delay(3000);

    if (konumsayac == 600)
    {
      printSerialData();
      myGsm.write(0x1A);
    }

    //   delay(3000);

    printSerialData();
    myGsm.flush();
    delay(onn);
  }

  else if ((msggelen.indexOf("+CGNSINF: 1,0,") > -1))
  {
    Serial.println("GPS Aktif Konum Bulunamadi lutfen bekleyiniz:");
    delay(onn);
  }
  else if ((msggelen.indexOf("+CGNSINF: 0,") > -1))
  {
    Serial.println("GPS Kapali Acilacak :");
    delay(onn);
  }
  else
  {
    msggelen = "";
    delay(onn);
  }
}

void setup()
{
  u8g2.begin(); // EKRAN KODU
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(5, 30, "YUKLENIYOR...");
  u8g2.sendBuffer();
  pinMode(manyet, OUTPUT);    // MANYETİK KİLİT
  digitalWrite(manyet, HIGH); // MANYETİK KİLİT

  pinMode(7, OUTPUT);
  delay(4000);
  digitalWrite(7, HIGH);
  delay(1000);
  Serial.begin(9600);

  myGsm.begin(9600);
  myGsm.print("AT+CGPSPWR=1\r\n"); // KONUM İÇİN EKLENDİ
  delay(500);
  myGsm.print("AT+CGNSSEQ=RMC\r\n"); // KONUM İÇİN EKLENDİ
  delay(1000);
  myGsm.println("AT+CIPSHUT"); // TCP VE UDP Dahil açık tüm bağlantıları kapatır.
  delay(1000);

  while (myGsm.available() != 0)
    Serial.write(myGsm.read());
  myGsm.println("AT+CGSN");
  delay(1000);
  myGsm.println("AT+CGATT=1"); // gprs aktif
  delay(2000);
  while (myGsm.available() != 0)
    Serial.write(myGsm.read());
  delay(2000);
  pinMode(k, OUTPUT); // MOTOR SÜRME
  pinMode(v, OUTPUT); // MOTOR SÜRME
  pinMode(5, INPUT);  // KAPI SENSÖRÜ
  pinMode(4, INPUT);  // KAPAK SENSÖRÜ

  Wire.begin(2);

  while (!Serial)
    ;
  Serial.println("I2C Scanner");

  if (!INA.begin())
  {
    Serial.println("could not connect. Fix and Reboot");
  }

  Serial.println("I2C Scanner");
  INA.setMaxCurrentShunt(1, 0.002);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(5, 30, "MESAFE SENSOR");
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(5, 55, "ALGILANMADI");
    u8g2.sendBuffer();
    msfinaktif = 1;
    delay(2000);
    goto o;
    while (1)
    {
    }
  }
o:

  sensor.startContinuous();
}

void printSerialData()
{
  while (myGsm.available() != 0)
    Serial.write(myGsm.read());
}

void readsimPort() // KONUM İÇİN EKLENDİ
{
  serialzaman = donguzaman;
  delay(bir);
  while ((donguzaman - serialzaman) < 200)
  {
    donguzaman = millis();
    while (myGsm.available())
    {
      donguzaman = millis();
      serialzaman = donguzaman;
      char c = myGsm.read();
      msggelen += c;
    }
  }
  myGsm.flush();
}

void readSerialPort() // KONUM İÇİN EKLENDİ
{
  while (Serial.available())
  {
    delay(onn);
    if (Serial.available() > 0)
    {
      char d = Serial.read();
      msggiden += d;
    }
  }
  Serial.flush();
}

void pres()
{
  i = 1;
  digitalWrite(manyet, LOW);
  sayacbir = sayacbir + 10;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(25, 30, "PRES");
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(5, 55, "YAPILIYOR");
  u8g2.sendBuffer();
  if (sayacbir < 2000)
  {
    digitalWrite(k, HIGH);
    digitalWrite(v, LOW);
  }
  else if (sayacbir >= 2000 && sayacbir < 2100)
  {
    digitalWrite(k, LOW);
    digitalWrite(v, LOW);
  }
  else if (sayacbir >= 2100 && sayacbir < 4100)
  {
    digitalWrite(k, LOW);
    digitalWrite(v, HIGH);
  }
  else if (sayacbir >= 4100)
  {
    digitalWrite(k, LOW);
    digitalWrite(v, LOW);

    i = 0;
    sayacbir = 0;
    // sayaciki = 0;
    presdoluluk = 0;
    ortdoluluk = 0;
    doluluksayac = 0;
    presikileme = presikileme + 1;
    digitalWrite(manyet, HIGH);
  }
}

void i2c()
{

  byte err, adr; // variable error is defined with address of I2C
  int number_of_devices;
  Serial.println("Scanning.");
  number_of_devices = 0;
  for (adr = 1; adr < 127; adr++)
  {
    Wire.beginTransmission(adr);
    err = Wire.endTransmission();

    if (err == 0)
    {
      Serial.print("I2C device at address 0x");
      if (adr < 16)
        Serial.print("0");
      Serial.print(adr, HEX);
      Serial.println("  !");
      number_of_devices++;
    }
    else if (err == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (adr < 16)
        Serial.print("0");
      Serial.println(adr, HEX);
    }
  }
  if (number_of_devices == 0)
    Serial.println("No I2C devices attached\n");
  else
    Serial.println("done\n");

  voltaj = INA.getBusVoltage();

  if (voltaj > 25.4 || voltaj < 19)
  {
    voltaj = 25.4;
  }

  c = sensor.readRangeContinuousMillimeters();
  // Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred())
  {
    Serial.print(" TIMEOUT");
  }

  if (c <= 1100 && c >= 0)
  {
    doluluk = (1100 - c + 250) / 11;
  }
  else if (c > 1100)
  {
    doluluk = 0;
  }
  if (doluluk > 99)
  {
    doluluk = 99;
  }
}

void term()
{
  if (sayacbes < 500)
  {
    s = analogRead(termPin);
    stp = s + stp;
    sayacbes = sayacbes + 10;
  }
  else
  {

    sayacbes = 0;
    sicaklik = (stp / 50) - 443;
    stp = 0;
  }
  if (sicaklik >= maxterm)
  {

    ihlalterm = ihlalterm + 10;
  }
  else
  {
    ihlalterm = 0;
  }

  if (sicaklik > 90 || sicaklik < 5)
  {
    sicaklik = 18;
  }
}

void sendimei()
{
  temp = "IMEI:869170035238840"; /// İMEİ GİR
  Serial.print("PIN = ");
  Serial.print(temp);
  // Serial.print("*C");
  Serial.println();
  // delay(5000);
  myGsm.println(temp);
  // delay(3000);
  printSerialData();
  myGsm.write(0x1A);
  // delay(3000);
  printSerialData();
}

void sendveriler()
{

  // delay(5000);
  myGsm.print("SeriNo:240/23-012");
  myGsm.print("Doluluk:");
  myGsm.println(doluluk);
  myGsm.print("Voltaj:");
  myGsm.println(voltaj);
  myGsm.print("Sıcaklık:");
  myGsm.println(sicaklik);
  myGsm.print("Kapı durum:");
  myGsm.println(kapidurum);
  myGsm.print("Kapak durum:");
  myGsm.println(kapakdurum);
  myGsm.print("İ:");
  myGsm.println(i);
  myGsm.print("Konum: ");
  myGsm.println(konum);
  // myGsm.print("Sayac iki:");myGsm.println( sayaciki);

  // delay(3000);
  printSerialData();
  myGsm.write(0x1A);
  // delay(3000);
  printSerialData();
}

void sims()
{
  simsayac = simsayac + 10;
  if (simsayac == 10)
  {
    sim808.connect(TCP, "185.98.62.213", 8181);
  }
  // delay(5000);
  // Serial.println("Connect ok success");
  if (simsayac == 500)
  {
    myGsm.println("AT+CIPSEND");
  }
  // delay(2000);
  if (simsayac == 700)
  {
    printSerialData();
    sendimei();
  }
  // delay(2000);
  if (simsayac == 900)
  {
    myGsm.println("AT+CIPSEND"); // veri göndermek için hazırla, cihazdan bir komut dönecek dönen komutun ardından veriyi yaz
  }
  // delay(3000);
  if (simsayac == 1200)
  {
    printSerialData();
    sendveriler();

    simsayac = 0;

    readsimPort();    // KONUM İÇİN EKLENDİ
    readSerialPort(); // KONUM İÇİN EKLENDİ
    gpskonum();       // KONUM İÇİN EKLENDİ
  }
}

void kapakdurumu()
{
  kapakdurum = digitalRead(kapak);
  if (kapakdurum == 1 && kapidurum == 1)
  {

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(30, 30, "KAPAK");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(30, 50, "ACILDI");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(48, 55, ".");
    u8g2.sendBuffer();
    ihlalkapak = ihlalkapak + 10;
  }
  else if (kapakdurum == 1 && kapidurum == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(3, 30, "KAPAK VE KAPI");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(30, 50, "ACILDI");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(48, 55, ".");
    u8g2.sendBuffer();
    ihlalkapak = ihlalkapak + 10;
  }
  else if (kapakdurum == 0 && kapidurum == 1)
  {
    ihlalkapak = 0;
  }
}

void kapidurumu()
{
  kapidurum = digitalRead(kapi);
  if (kapidurum == 0 && kapakdurum == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(40, 30, "KAPI");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(30, 50, "ACILDI");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(48, 55, ".");
    u8g2.sendBuffer();
    ihlalkapi = ihlalkapi + 10;
  }
  else if (kapakdurum == 1 && kapidurum == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(3, 30, "KAPAK VE KAPI");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(30, 50, "ACILDI");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(48, 55, ".");
    u8g2.sendBuffer();
    ihlalkapi = ihlalkapi + 10;
  }
  else if (kapakdurum == 0 && kapidurum == 1)
  {
    ihlalkapi = 0;
  }
}

void lcd()
{
  if (lcdsayaci > 0 && lcdsayaci < 400 && kapakdurum == 0 && kapidurum == 1 && i == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_logisoso34_tf);
    u8g2.drawStr(5, 41, "PENDIK");
    u8g2.setFont(u8g2_font_logisoso34_tf);
    u8g2.drawStr(90, 5, ".");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(2, 64, "BELEDIYESI");
    u8g2.setFont(u8g2_font_logisoso34_tf);
    u8g2.drawStr(67, 48, ".");
    u8g2.drawStr(116, 48, ".");
    u8g2.sendBuffer();
  }
  if (lcdsayaci >= 400 && lcdsayaci < 800 && kapakdurum == 0 && kapidurum == 1 && i == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 20, "SICAKLIK ");
    u8g2.setFont(u8g2_font_ncenB24_tr);
    u8g2.setCursor(0, 55);
    u8g2.print(sicaklik);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(40, 35, "o");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(46, 45, "C");

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(69, 15, "DOLULUK");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(69, 25, "ORANI");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(115, 25, "%");
    u8g2.setFont(u8g2_font_ncenB24_tr);
    u8g2.setCursor(80, 55);
    u8g2.print(doluluk);

    u8g2.sendBuffer();
  }
  if (lcdsayaci >= 800)
  {
    lcdsayaci = 0;
  }
}

void loop()
{
  lcdsayaci = lcdsayaci + 10;
  doluluksayac = doluluksayac + 10;
  delay(10);

  sayacmsf = sayacmsf + 10;

  sims();
  term();
  kapakdurumu();
  kapidurumu();
  lcd();

  if (sayacuc >= 200) // mesafe ve voltaj okur. 2sn de bir okunur.
  {
    // vl53l0x ve ina226 sensörleri
    i2c();
    ortdoluluk = ortdoluluk + doluluk;
    sayacuc = 0;
  }
  else
  {
    sayacuc = sayacuc + 10;
  }

  if (doluluksayac == 18000) // 1 dk lık doluluk bilgisinin ortalamasını alır.
  {
    presdoluluk = ortdoluluk / 90;
  }

  if (sayacdort >= 200)
  { // 2 sn bir serial den yazı yazar.
    Serial.print("voltaj = ");
    Serial.println(voltaj);
    Serial.print("mesafe = ");
    Serial.println(c);
    Serial.print("sicaklik = ");
    Serial.println(sicaklik);
    Serial.print("kapidurum = ");
    Serial.println(kapidurum);
    Serial.print("kapakdurum = ");
    Serial.println(kapakdurum);
    Serial.print("lcdsayaci = ");
    Serial.println(lcdsayaci);
    Serial.print("sayaciki = ");
    Serial.println(sayaciki);
    Serial.print("doluluk = ");
    Serial.println(doluluk);
    Serial.print("i = ");
    Serial.println(i);
    Serial.print("presdoluluk = ");
    Serial.println(presdoluluk);
    Serial.print("presdoluluk = ");
    Serial.println(presdoluluk);
    if (i == 1)
    {
      Serial.println("Manyetik Kilit Açık");
    }
    else
    {
      Serial.println("Manyetik Kilit Kapalı");
    }
    sayacdort = 0;
  }
  else
  {
    sayacdort = sayacdort + 10;
  }

  if (presikileme < 2) // PRES
  {
    if (presdoluluk > 70 && ihlalkapak < 20 && ihlalkapi < 20) // 3 dk lık ortalama doluluk değeri belirli bir değerden(70) fazla ise pres yaptır.
    {
      pres();
    }
    else
    {
      // sayaciki = sayaciki + 10;
      digitalWrite(manyet, HIGH);
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
      i = 0;
    }
  }

  if (ihlalkapi > 20) // kapi açılırsa atık boşaltılmış kabul eder ve presikilemeyi sıfırlar.
  {
    presikileme = 0;
    presdoluluk = 0;
    ortdoluluk = 0;
    doluluksayac = 0;
  }

  if (sayacmsf > 4320000 && msfinaktif == 1) // Mesafe sensörü algılamaz ise 12 saatte bir pres yapar.
  {
    sayacmsf = 0;
    pres();
  }
  else
  {
    digitalWrite(manyet, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    i = 0;
  }

  if (sayacmsf > 4320000 && msfinaktif == 0)
  {
    sayacmsf = 0;
  }
}
