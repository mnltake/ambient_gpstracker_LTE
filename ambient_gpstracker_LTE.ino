#include <Arduino.h>
#include <M5StickC.h>
#include "AmbientGsm.h"
#include <TinyGPSPlus.h>
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>
#define SerialMon Serial
#define SerialAT Serial1
#define SerialGPS Serial2
TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
TinyGPSPlus gps;

#define PERIOD 30

const char apn[]  = "povo.jp";
const char gprsUser[] = "";
const char gprsPass[] = "";

unsigned int channelId = 100; // AmbientのチャネルID
const char* writeKey = "writeKey"; // ライトキー
Ambient ambient;

double lat=0.0, lng=0.0 ,alt;

void setup() {
  M5.begin();//default baud rate 115200
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(1);
  delay(200);
  SerialMon.println("Wait...");

  // Set GSM module baud rate
  SerialAT.begin(115200, SERIAL_8N1, 33, 32);//M5StickC to CAT-M UNIT
  SerialGPS.begin(115200, SERIAL_8N1, 36, 26);//GPIO36 rx only to GPS Receiver UART
  delay(100);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  //modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print(F("waitForNetwork()"));
  while (!modem.waitForNetwork()) SerialMon.print(".");
  SerialMon.println(F(" Ok."));

  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  modem.gprsConnect(apn, gprsUser, gprsPass);
  SerialMon.println(F(" done."));

  SerialMon.print(F("isNetworkConnected()"));
  while (!modem.isNetworkConnected()) SerialMon.print(".");
  SerialMon.println(F(" Ok."));

  SerialMon.print(F("My IP addr: "));
  IPAddress ipaddr = modem.localIP();
  SerialMon.println(ipaddr);
  M5.Lcd.setCursor(5,0);
  M5.Lcd.println(ipaddr);

  ambient.begin(channelId, writeKey, &client); 

}

void loop(){
    //

    unsigned long t = millis();
    float uptime;
    // 起動時間を送信する
    uptime = (float)millis()/1000;
    ambient.set(1, String(uptime).c_str());// 1番目のデーターとしてuptimeをセット
    
    while (SerialGPS.available() > 0)
      if (gps.encode(SerialGPS.read()))
          if (gps.location.isValid()){
            lat = gps.location.lat();
            lng = gps.location.lng();
            Serial.print(lat, 11);
            Serial.print(F(","));
            Serial.print(lng, 11);
          }else{
            Serial.print(F("INVALID"));
          }
          if (gps.altitude.isValid()){
            alt = gps.altitude.meters();
            Serial.print(alt,7);
          }else{
            Serial.print(F("INVALID"));
          } 
          Serial.println();
    char latbuf[12], lngbuf[12],altbuf[8];
    if (!(lat || lng)){
      dtostrf(lat, 11, 7, latbuf);
      ambient.set(9, latbuf); // 9番目のデーターとして緯度をセット
      dtostrf(lng, 11, 7, lngbuf);
      ambient.set(10, lngbuf); // 10番目のデーターとして経度をセット
      dtostrf(alt, 7 ,2, altbuf);
      ambient.set(2, altbuf); // 2番目のデーターとして高さをセット
    }
    M5.Lcd.fillScreen(BLACK);

    M5.Lcd.setCursor(5,15);
    M5.Lcd.printf("uptime:%.1f", uptime);
    M5.Lcd.setCursor(5,30);
    M5.Lcd.printf("lat:%.6f", lat);
    M5.Lcd.setCursor(5,45);
    M5.Lcd.printf("lng:%.6f", lng);
    M5.Lcd.setCursor(5,60);
    M5.Lcd.printf("alt:%.2f", alt);
    modem.gprsConnect("povo.jp", "", "");
    while (!modem.isNetworkConnected()) SerialMon.print(".");
    SerialMon.print(F("My IP addr: "));
    IPAddress ipaddr = modem.localIP();
    SerialMon.println(ipaddr);
    M5.Lcd.setCursor(5,0);
    M5.Lcd.println(ipaddr);
    SerialMon.println(" connect");
    (ambient.send())? Serial.println("send ok") : Serial.println("send error");

    t = millis() - t;
    t = (t < PERIOD * 1000) ? (PERIOD * 1000 - t) : 1;
    delay(t);
}
