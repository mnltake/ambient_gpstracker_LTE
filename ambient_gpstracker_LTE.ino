/*
  Use ESP32 WiFi to get RTCM data from RTK2Go (caster) as a Client, and transmit GGA (needed for some Casters)
  By: SparkFun Electronics / Nathan Seidle
  Date: November 18th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain RTCM data from a NTRIP Caster over WiFi
  and push it over I2C to a ZED-F9x.
  It's confusing, but the Arduino is acting as a 'client' to a 'caster'. In this case we will
  use RTK2Go.com as our caster because it is free. See the NTRIPServer example to see how
  to push RTCM data to the caster.

  The rover's location will be broadcast to the Caster every 10s via GGA setence.

  You will need to have a valid mountpoint available. To see available mountpoints go here: http://rtk2go.com:2101/

  This is a proof of concept to show how to connect to a caster via HTTP.

  For more information about NTRIP Clients and the differences between Rev1 and Rev2 of the protocol
  please see: https://www.use-snip.com/kb/knowledge-base/ntrip-rev1-versus-rev2-formats/

  "In broad protocol terms, the NTRIP client must first connect (get an HTTP “OK” reply) and only then
  should it send the sentence.  NTRIP protocol revision 2 (which does not have very broad industry
  acceptance at this time) does allow sending the sentence in the original header."
  https://www.use-snip.com/kb/knowledge-base/subtle-issues-with-using-ntrip-client-nmea-183-strings/

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/18443
  RTK Express: https://www.sparkfun.com/products/18442

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

//#include <WiFi.h>

#define SerialMon Serial
#define SerialGPS Serial1
#define SerialAT Serial2
// #define DUMP_AT_COMMANDS
// #define TINY_GSM_DEBUG Serial

#define TINY_GSM_RX_BUFFER 650
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>
TinyGsm modem(SerialAT); /* LTE-M modem */
TinyGsmClient ntripClient(modem, 0);
TinyGsmClient ambientClient(modem, 1);
TinyGsmClientSecure geoidClient(modem, 2);
#include "secrets.h"
//#include <M5Stack.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include "AmbientGsm.h"
Ambient ambient;
#include <ArduinoHttpClient.h>
HttpClient    http(geoidClient,"vldb.gsi.go.jp" , 443); //https://vldb.gsi.go.jp/sokuchi/surveycalc/api_help.html
#include <ArduinoJson.h>
#include "base64.h"
#include <TimeLib.h>  
#include <Avatar.h>
using namespace m5avatar;
Avatar avatar;

#define E7 10000000.0
#define E3 1000.0
#include "esp_system.h"


//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0;       //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

bool transmitLocation = true;        //By default we will transmit the units location via GGA sentence.
int timeBetweenGGAUpdate_ms = 5000; //GGA is required for Rev2 NTRIP casters. Don't transmit but once every 10 seconds
long lastTransmittedGGA_ms = 0;

//Used for GGA sentence parsing from incoming NMEA
bool ggaSentenceStarted = false;
bool ggaSentenceComplete = false;
bool ggaTransmitComplete = false; //Goes true once we transmit GGA to the caster

char ggaSentence[128] = {0};
byte ggaSentenceSpot = 0;
int ggaSentenceEndSpot = 0;
unsigned int ambientcnt=0;
const int ambientSendPeriod =8;
#define AMBUFSIZE  1460
char ambuffer[AMBUFSIZE];
ColorPalette* cps[4];
bool isGetgeoid = false;
float  geoid;
const int wdtTimeout = 20000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void beginClient();

double localGeoid(double _lng_mdeg){
  return (_lng_mdeg * 4.3144 -552.65*E7)/10000.0;
} 

float getJapanGeoid(){
  char latbuf[12], lngbuf[12],geobuf[8],request[200];
  dtostrf(myGNSS.getLatitude()/ E7, -1, 8, latbuf);
  dtostrf(myGNSS.getLongitude() / E7, -1, 8, lngbuf);
  sprintf(request,"/sokuchi/surveycalc/geoid/calcgh/cgi/geoidcalc.pl?outputType=json&latitude=%s&longitude=%s",latbuf,lngbuf);
  // Serial.println(request);
  SerialMon.print(F("Performing HTTP GET request... "));
  int err = http.get(request);
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(10000);
    isGetgeoid = false;
    return 0.0;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(10000);
    isGetgeoid = false;
    return 0.0;
  }

  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    // SerialMon.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
    SerialMon.print(F("Content length is: "));
    SerialMon.println(length);
  }
  if (http.isResponseChunked()) {
    SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  SerialMon.print(F("Body length is: "));
  SerialMon.println(body.length());

  // Allocate the JSON document
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  // const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(2) + 60;
  DynamicJsonDocument doc(192);
  // Parse JSON object
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    isGetgeoid = false;
    return 0.0;
  }

  JsonObject OutputData = doc["OutputData"];
  // const char* OutputData_latitude = OutputData["latitude"]; // "34.953764000"
  // const char* OutputData_longitude = OutputData["longitude"]; // "136.935113500"
  const char* OutputData_geoidHeight = OutputData["geoidHeight"]; // "38.1422"
  Serial.println(OutputData_geoidHeight);
  isGetgeoid = true;
  return atof(OutputData_geoidHeight)*E3;

  // Disconnect
  // geoidClient.stop();


}

void IRAM_ATTR resetModule() {
  ets_printf("WDT reboot\n");
  ntripClient.stop();
  ambientClient.stop();
  geoidClient.stop();
  esp_restart();
}

void setup()
{
  M5.begin(true, true, true, false); 
  dacWrite(25, 0);  // speaker off
  #ifdef DUMP_AT_COMMANDS
    #include <StreamDebugger.h>
    StreamDebugger debugger(SerialAT, SerialMon);
    TinyGsm modem(debugger);
  #else
    TinyGsm modem(SerialAT);
  #endif
    //watchdog timer
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  avatar.init(); // start drawing
  cps[0] = new ColorPalette();
  cps[1] = new ColorPalette();
  cps[2] = new ColorPalette();
  cps[3] = new ColorPalette();
  cps[1]->set(COLOR_PRIMARY, TFT_ORANGE);
  cps[1]->set(COLOR_BACKGROUND, TFT_BLACK);
  cps[2]->set(COLOR_PRIMARY, TFT_GREEN);
  cps[2]->set(COLOR_BACKGROUND, TFT_BLACK);
  cps[3]->set(COLOR_PRIMARY, TFT_RED);
  cps[3]->set(COLOR_BACKGROUND, TFT_BLACK);
  avatar.setColorPalette(*cps[0]);
  avatar.setSpeechText("Starting..");
  // // // M5.Lcd.setTextSize(3);
  // // // M5.Lcd.setCursor(0,0);
  Serial.println(F("NTRIP start"));
  SerialGPS.begin(115200, SERIAL_8N1, 36, 26);
  
  delay(300);
  while (!myGNSS.begin(SerialGPS)) {
    Serial.print(".");delay(100);
  }
  timerWrite(timer, 0);
  Serial.println("GNSS serial connected");
  myGNSS.setUART1Output(COM_TYPE_UBX);                                //Set the UART1 port to output both NMEA and UBX messages
  myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
  myGNSS.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA messages
  //myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); //Verify the GGA sentence is enabled
 // myGNSS.setAutoPVT(true);
  myGNSS.setNavigationFrequency(1); //Set output in Hz.
  myGNSS.setMeasurementRate(1000); //Produce a measurement every 1000ms
  myGNSS.setNavigationRate(1); //Produce a navigation solution every measurement
  myGNSS.setAutoPVTrate(1); //Tell the GNSS to send the PVT solution every measurement
  avatar.setSpeechText("Connect..");
  SerialAT.begin(115200, SERIAL_8N1, 17, 16);
  delay(1800);
  timerWrite(timer, 0);
  modem.sendAT();modem.sendAT();modem.sendAT();
  modem.restart();
  String modemInfo = modem.getModemInfo();
  Serial.println( modemInfo );
  avatar.setMouthOpenRatio(0.5);
  Serial.print( "connecting " );Serial.println( apn );
  timerWrite(timer, 0);
  while (!modem.waitForNetwork()) Serial.print(".");
  modem.gprsConnect(apn, gprsUser, gprsPass);
  while (!modem.isNetworkConnected()) Serial.print(".");
  IPAddress ipaddr = modem.localIP();
  Serial.println( ipaddr );
  avatar.setMouthOpenRatio(0.0);
   // // // M5.Lcd.println( ipaddr);

  ambient.begin(channelId, writeKey, &ambientClient);
  sprintf(ambuffer, "{\"writeKey\":\"%s\",\"data\":[", writeKey);
}

void loop(){
  timerWrite(timer, 0);
  beginClient();
  delay(5);
   
}

//Connect to NTRIP Caster, receive RTCM, and push to ublox module over UART1
void beginClient()
{
//  WiFiClient ntripClient;
  long rtcmCount = 0;
  myGNSS.checkUblox();
  avatar.setMouthOpenRatio(0.0);
  //Connect if we are not already. Limit to 5s between attempts.
  if (ntripClient.connected() == false)
  {
    Serial.print(F("Opening socket to "));
    Serial.println(casterHost);

    if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
    {
      Serial.println(F("Connection to caster failed"));
      return;
    }
    else
    {
      Serial.print(F("Connected to "));
      Serial.print(casterHost);
      Serial.print(F(": "));
      Serial.println(casterPort);

      Serial.print(F("Requesting NTRIP Data from mount point "));
      Serial.println(mountPoint);

      size_t SERVER_BUFFER_SIZE = 512;
      char serverRequest[SERVER_BUFFER_SIZE];

      snprintf(serverRequest,
              SERVER_BUFFER_SIZE,
              "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
              mountPoint);

      char credentials[512];
      if (strlen(casterUser) == 0)
      {
        strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
      }
      else
      {
        //Pass base64 encoded user:pw
        char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
        snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

        Serial.print(F("Sending credentials: "));
        Serial.println(userCredentials);

        //Encode with ESP32 built-in library
        base64 b;
        String strEncodedCredentials = b.encode(userCredentials);
        char encodedCredentials[strEncodedCredentials.length() + 1];
        strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
        snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
      }
      strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
      strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

      Serial.print(F("serverRequest size: "));
      Serial.print(strlen(serverRequest));
      Serial.print(F(" of "));
      Serial.print(sizeof(serverRequest));
      Serial.println(F(" bytes available"));

      Serial.println(F("Sending server request:"));
      Serial.println(serverRequest);
//    ntripClient.write(serverRequest, strlen(serverRequest));
      ntripClient.print(serverRequest);
      
      //Wait for response
      unsigned long timeout = millis();
      while (ntripClient.available() == 0)
      {
        if (millis() - timeout > 5000)
        {
          Serial.println(F("Caster timed out!"));
          ntripClient.stop();
          return;
        }
        delay(10);
      }

      //Check reply
      bool connectionSuccess = false;
      char response[512];
      int responseSpot = 0;
      while (ntripClient.available())
      {
        if (responseSpot == sizeof(response) - 1)
          break;

        response[responseSpot++] = ntripClient.read();
        if (strstr(response, "200") > 0) //Look for '200 OK'
          connectionSuccess = true;
        if (strstr(response, "401") > 0) //Look for '401 Unauthorized'
        {
          Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
          connectionSuccess = false;
        }
      }
      response[responseSpot] = '\0';

      Serial.print(F("Caster responded with: "));
      Serial.println(response);

      if (connectionSuccess == false)
      {
        Serial.print(F("Failed to connect to "));
        Serial.println(casterHost);
        return;
      }
      else
      {
        Serial.print(F("Connected to "));
        Serial.println(casterHost);
        lastReceivedRTCM_ms = millis(); //Reset timeout
        ggaTransmitComplete = true;     //Reset to start polling for new GGA data
      }
    } //End attempt to connect
  }   //End connected == false

  if (ntripClient.connected() == true)
  {
    uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    rtcmCount = 0;
    
    //Print any available RTCM data
    while (ntripClient.available())
    {
      //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
      avatar.setMouthOpenRatio(0.7);
      rtcmData[rtcmCount++] = ntripClient.read();
      if (rtcmCount == sizeof(rtcmData))
        break;
    }
    avatar.setMouthOpenRatio(0.0);
    if (rtcmCount > 0)
    {
    // lastReceivedRTCM_ms = millis();
     
      //Push RTCM to GNSS module over I2C
      myGNSS.pushRawData(rtcmData, rtcmCount, false);
      Serial.print(F("RTCM pushed to Ublox: "));
      Serial.println(rtcmCount);
      // NTRIP bytes per second
      
      unsigned long milis = millis();
      unsigned long spanMillis = milis - lastReceivedRTCM_ms;
      if ( spanMillis > 0 ){
        int bps = rtcmCount * 8 * 1000 / spanMillis;
        lastReceivedRTCM_ms = milis;
        // // M5.Lcd.setCursor(0,25);
        // // M5.Lcd.printf("NTRIP=%05d bps\r\n", bps );
        }
    }
  }
  
  //Provide the caster with our current position as needed
  if (ntripClient.connected() == true && transmitLocation == true && (millis() - lastTransmittedGGA_ms) > timeBetweenGGAUpdate_ms)// && ggaSentenceComplete == true && ggaTransmitComplete == false)
  {
    lastTransmittedGGA_ms = millis();
    ggaTransmitComplete = true;
    char latbuf[12], lngbuf[12],altbuf[8],spdbuf[7],avatartalk[20],altMSLbuf[8],geoidbuf[8],innergeoidbuf[8];
    M5.update();
    if (myGNSS.getPVT() && ((myGNSS.getLatitude() > 34*E7) && (myGNSS.getLatitude() < 38*E7)) ){
        double latitude_mdeg =myGNSS.getLatitude();
        double longitude_mdeg = myGNSS.getLongitude();
        double alt = myGNSS.getAltitude();
        double inner_altMSL = myGNSS.getAltitudeMSL();
        
        if (!isGetgeoid){
          geoid = localGeoid(longitude_mdeg);
        } 
        double altMSL = alt-geoid;
        double innner_geoid = alt - inner_altMSL;
        double speed = myGNSS.getGroundSpeed() ;
        int fixType = myGNSS.getFixType();  //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
        int rtk = myGNSS.getCarrierSolutionType(); //0=No solution, 1=Float solution, 2=Fixed solution}
          //local time set  
        int YYYY=myGNSS.getYear();
        int MM = myGNSS.getMonth();
        int DD = myGNSS.getDay();
        int HH = myGNSS.getHour();
        int mm = myGNSS.getMinute();
        int ss = myGNSS.getSecond();
        setTime(HH, mm, ss, DD, MM, YYYY);
        adjustTime(+9 * SECS_PER_HOUR);
        dtostrf(longitude_mdeg / E7, -1, 7, lngbuf);
        dtostrf(latitude_mdeg/ E7, -1, 7, latbuf);
        dtostrf(alt/E3, -1 ,2, altbuf);
        dtostrf(altMSL/E3, -1 ,2, altMSLbuf);
        dtostrf(speed /E3, -1 ,2, spdbuf);
        dtostrf(geoid/E3, -1 ,2, geoidbuf);
        dtostrf(innner_geoid/E3, -1 ,2, innergeoidbuf);
        //YYYY-MM-DD HH:mm:ss.sss”
        char datebuf[25];
        sprintf(datebuf,"%04d-%02d-%02d %02d:%02d:%02d",
            year(),month(),day(),hour(),minute(),second());
 
        if (ambientcnt){ //最初の一回目は使わない
        sprintf(&ambuffer[strlen(ambuffer)], "{\"created\":\"%s\",\"d1\":\"%s\",\"d2\":\"%s\",\"d3\":\"%d\",\"d5\":\"%d\",\"d6\":\"%s\",\"d7\":\"%s\",\"d8\":\"%s\",\"lat\":\"%s\",\"lng\":\"%s\"},",
                        datebuf ,altMSLbuf,spdbuf,ambientcnt,rtk,altbuf,geoidbuf,innergeoidbuf,latbuf,lngbuf);
        }
        // synchronizeAllServosStartAndWaitForAllServosToStop();

        if (M5.BtnA.wasPressed()){
          sprintf(avatartalk,"Lat%s" , latbuf);
        }else if (M5.BtnB.wasPressed()){
          sprintf(avatartalk,"Lng%s" , lngbuf);
        }else if (M5.BtnC.wasPressed()){
          Serial.println("get japan geoid");
          geoid = getJapanGeoid();
          sprintf(avatartalk,"Get geoid%.2f" ,geoid/E3);
        }else{
        sprintf(avatartalk,"AltMSL%sm" , altMSLbuf);
        // sprintf(avatartalk,"SoracomUG" );
        }
        avatar.setSpeechText(avatartalk);
        if (!isGetgeoid){
          avatar.setColorPalette(*cps[3]);
        }else{
          switch (rtk)
          {
          case 1/* constant-expression */:
            avatar.setColorPalette(*cps[1]);
            // // M5.Lcd.println("Float");
            break;
          case 2/* constant-expression */:
            avatar.setColorPalette(*cps[2]);
            // // M5.Lcd.println("  Fix");
            break;
          default:
            // // M5.Lcd.println("NORTK");
            avatar.setColorPalette(*cps[0]);
            break;
          }
        }
    }
    // if (ambientcnt > ambientSendPeriod){
    if (((ambientcnt % ambientSendPeriod) == 0)&& ambientcnt ){
      ambuffer[strlen(ambuffer)-1] = '\0';
      sprintf(&ambuffer[strlen(ambuffer)], "]}\r\n");
      Serial.write(ambuffer,strlen(ambuffer));
      avatar.setMouthOpenRatio(1.0);
      int r = ambient.bulk_send(ambuffer,5000);
      avatar.setMouthOpenRatio(0.0);
      Serial.println(r);
      if (r){
        ambientcnt++;
        Serial.println("ambient send");
        avatar.setSpeechText("SendData");
        sprintf(ambuffer, "{\"writeKey\":\"%s\",\"data\":[", writeKey);
        return;
      }else{
        Serial.println("Ambient send error");
        ambientcnt--;
        return;
      }

    }else{
      if (ambientcnt %60 == 1){
        timerWrite(timer, 0);
        geoid = getJapanGeoid();
      }
      ambientcnt++;
    }
  }

  //Close socket if we don't have new data for 10s
  if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
  {
    Serial.println(F("RTCM timeout. Disconnecting..."));
    if (ntripClient.connected() == true)
      ntripClient.stop();
    return;
  }

  delay(10);
}
