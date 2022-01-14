#include <Arduino.h>
#include <M5StickC.h>
#include "AmbientGsm.h"
//#include <TinyGPSPlus.h>
#include <TinyGPS++.h>
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>
#define SerialMon Serial
#define SerialAT Serial1
#define SerialGPS Serial2
#define DEEPSLEEP_SEC 100
#define ACTIVE_SEC 10

TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
TinyGPSPlus gps;
Ambient ambient;

RTC_DATA_ATTR int sendCount = 0;
const char apn[]  = "povo.jp";
const char gprsUser[] = "";
const char gprsPass[] = "";

unsigned int channelId = 100; // AmbientのチャネルID
const char* writeKey = "write key"; // ライトキー

double lat=0.0, lng=0.0 ,alt ,spd=0.0;
char latbuf[12], lngbuf[12],altbuf[8],spdbuf[7];
int sleepCount =0;

void deepsleep(){
  modem.gprsDisconnect();
  modem.poweroff();
  SerialMon.println(F("GPRS disconnected"));
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_SEC*1000000UL);//usec
  // ディープスリープ
  SerialMon.println("deepsleepstart");
  M5.Lcd.setCursor(5,70);
  M5.Lcd.print("deep sleep");
  Serial.flush(); 
  esp_deep_sleep_start();
}

void setup() {
  M5.begin();//default baud rate 115200
  M5.Axp.ScreenBreath(10); 
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(1);
  delay(200);
  SerialMon.println("Wait...");
  M5.Lcd.println("Wait...");
  // Set GSM module baud rate
  SerialAT.begin(115200, SERIAL_8N1, 33, 32);//M5StickC to CAT-M UNIT
  SerialGPS.begin(115200, SERIAL_8N1, 36, 26);//GPIO36 rx only to GPS Receiver UART
  delay(100);
  ambient.begin(channelId, writeKey, &client);
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
// PSM 1min sleep 3sec active
//  modem.sendAT("+CPSMS=1,,,\"01111110\",\"00000011\""); 
  modem.sendAT("+CPSMS=0");//disable PSM
}

void loop(){
  while (SerialGPS.available() > 0){
    if (gps.encode(SerialGPS.read())){
        if (gps.location.isValid()){
          lat = gps.location.lat();
          lng = gps.location.lng();
          SerialMon.print(lat, 11);
          SerialMon.print(F(","));
          SerialMon.print(lng, 11);
        }else{
          SerialMon.print(F("lon lat INVALID"));
        }
        if (gps.altitude.isValid()){
          alt = gps.altitude.meters();
          SerialMon.print(alt,7);
        }else{
          SerialMon.print(F("alt INVALID"));
        } 
        SerialMon.println();
        if (gps.speed.isValid()){
          spd = gps.speed.kmph();
          SerialMon.print(spd,6);
        }else{
          SerialMon.print(F("speed INVALID"));
        } 
        SerialMon.println();
    }
  }
  if (lat && lng){ //測位できたとき  
    ambient.set(1, String(sendCount).c_str());// 1番目のデーターとしてbootCountをセット
    dtostrf(alt, 7 ,2, altbuf);
    ambient.set(2, altbuf); // 2番目のデーターとして高さをセット
    dtostrf(spd, 6 ,2, spdbuf);
    ambient.set(3, spdbuf); // 3番目のデーターとしてspeedをセット
    dtostrf(lat, 11, 7, latbuf);
    ambient.set(9, latbuf); // 9番目のデーターとして緯度をセット
    dtostrf(lng, 11, 7, lngbuf);
    ambient.set(10, lngbuf); // 10番目のデーターとして経度をセット
    if (spd < 0.1){
      sleepCount ++;
    }else{
      sleepCount=0;
    }
    if (sleepCount > 6){
      deepsleep();
    }

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(5,15);
    M5.Lcd.printf("sendCount:%d", sendCount);
    M5.Lcd.setCursor(5,30);
    M5.Lcd.printf("lat:%.7lf", lat);
    M5.Lcd.setCursor(5,45);
    M5.Lcd.printf("lng:%.7lf", lng);
    M5.Lcd.setCursor(5,60);
    M5.Lcd.printf("alt:%.2lf m", alt);
    M5.Lcd.printf("spd:%.2lf km/h", spd);
    
    while (!modem.isNetworkConnected()) {
      SerialMon.print("conecting..");
      modem.waitForNetwork();
      modem.gprsConnect(apn, gprsUser, gprsPass);
      delay(3000);
    }
    SerialMon.print(F("My IP addr: "));
    IPAddress ipaddr = modem.localIP();
    SerialMon.println(ipaddr);
    M5.Lcd.setCursor(5,0);
    M5.Lcd.print(ipaddr);
    if (ipaddr){
      SerialMon.println(" connect");
      if (ambient.send()){
        SerialMon.println("send ok"); 
        M5.Lcd.print(" send ok");
        sendCount++;
        delay(ACTIVE_SEC*1000);
      }else{
        SerialMon.println("send error");
        M5.Lcd.print(" send error");
      }
    }
  }else{
    sleepCount++;
  }
}
