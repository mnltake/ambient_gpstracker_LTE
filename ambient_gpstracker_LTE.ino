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
#define SerialGPS Serial1
#define SerialAT Serial2
#define TINY_GSM_RX_BUFFER 650
#define TINY_GSM_MODEM_SIM7080
#include "TinyGsmClient.h"
TinyGsm modem(SerialAT); /* LTE-M modem */
TinyGsmClient ntripClient(modem);
#include "secrets.h"
#include <M5Stack.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include "AmbientGsm.h"
Ambient ambient;

#include "base64.h"
#include <TimeLib.h>  
#include <Avatar.h>
using namespace m5avatar;
Avatar avatar;

#define E7 10000000.0
#define E3 1000.0
#ifdef USESERVO
  #include <servoeasing.hpp>
  #define SERVO_PIN_X 5
  #define SERVO_PIN_Y 2
  #define START_DEGREE_VALUE_X 90
  #define START_DEGREE_VALUE_Y 90
  ServoEasing servo_x;
  ServoEasing servo_y;
#endif
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
int ambientcnt=0;
const int ambientSendPeriod = 6;
#define AMBUFSIZE  1400
char ambuffer[AMBUFSIZE];
ColorPalette* cps[4];
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void beginClient();

void setup()
{
  M5.begin(true, true, true, false); 
  dacWrite(25, 0);  // speaker off
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
  Serial.println(F("NTRIP testing"));

  while (!myGNSS.begin(SerialGPS)) {
    SerialGPS.begin(115200, SERIAL_8N1, 36, 26);
    Serial.print(".");delay(100);
  }

  Serial.println("GNSS serial connected");
  myGNSS.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);                                //Set the UART1 port to output both NMEA and UBX messages
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
  modem.restart();
  String modemInfo = modem.getModemInfo();
  Serial.println( modemInfo );
  avatar.setMouthOpenRatio(0.5);
  Serial.print( "connecting " );Serial.println( apn );
  while (!modem.waitForNetwork()) Serial.print(".");
  modem.gprsConnect(apn, gprsUser, gprsPass);
  while (!modem.isNetworkConnected()) Serial.print(".");
  IPAddress ipaddr = modem.localIP();
  Serial.println( ipaddr );
  avatar.setMouthOpenRatio(0.0);
   // // // M5.Lcd.println( ipaddr);

  ambient.begin(channelId, writeKey, & ntripClient);
  sprintf(ambuffer, "{\"writeKey\":\"%s\",\"data\":[", writeKey);
  
  #ifdef USESERVO
    if (servo_x.attach(SERVO_PIN_X, START_DEGREE_VALUE_X, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
      Serial.print("Error attaching servo x");
    }
    if (servo_y.attach(SERVO_PIN_Y, START_DEGREE_VALUE_Y, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
      Serial.print("Error attaching servo y");
    }
    servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
    servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
    setSpeedForAllServos(60);
    synchronizeAllServosStartAndWaitForAllServosToStop();
  #endif USESERVO

}

void loop(){
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

      const int SERVER_BUFFER_SIZE = 512;
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
  if (ntripClient.connected() == true && transmitLocation == true && (millis() - lastTransmittedGGA_ms) > timeBetweenGGAUpdate_ms && ggaSentenceComplete == true && ggaTransmitComplete == false)
  {
    Serial.print(F("Pushing GGA to server: "));
    Serial.println(ggaSentence);
    char latbuf[12], lngbuf[12],altbuf[8],spdbuf[7],avatartalk[20];
    M5.update();
    if (myGNSS.getPVT() && myGNSS.getLatitude() ){
        double latitude_mdeg =myGNSS.getLatitude();
        double longitude_mdeg = myGNSS.getLongitude();
        double alt = myGNSS.getAltitude();
        double altMSL = myGNSS.getAltitudeMSL();
        double geoid = alt - altMSL;
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
        dtostrf(longitude_mdeg / E7, 11, 7, lngbuf);
        dtostrf(latitude_mdeg/ E7, 11, 7, latbuf);
        dtostrf(altMSL/E3, 7 ,2, altbuf);
        dtostrf(speed /E3, 6 ,2, spdbuf);
        //YYYY-MM-DD HH:mm:ss.sss”
        char datebuf[25];
        sprintf(datebuf,"%04d-%02d-%02d %02d:%02d:%02d.000",
            year(),month(),day(),hour(),minute(),second());
        
        sprintf(&ambuffer[strlen(ambuffer)], "{\"created\":\"%s\",\"d1\":\"%s\",\"d2\":\"%s\",\"d3\":\"%s\",\"d4\":\"%s\",\"d5\":\"%d\",\"lat\":\"%s\",\"lng\":\"%s\"},",
                        datebuf ,altbuf,spdbuf,latbuf,lngbuf,rtk,latbuf,lngbuf);
        // synchronizeAllServosStartAndWaitForAllServosToStop();

        if (M5.BtnA.wasPressed()){
        sprintf(avatartalk,"Lat%s" , latbuf);
        }else if (M5.BtnB.wasPressed()){
        sprintf(avatartalk,"Lng%s" , lngbuf);
        }else if (M5.BtnC.wasPressed()){
        sprintf(avatartalk,"Speed%s" , spdbuf);
        }else{
        sprintf(avatartalk,"Alt%sm" , altbuf);
        }
        avatar.setSpeechText(avatartalk);
        
        if (abs((longitude_mdeg / E7 -POINTLNG) < DIFF ) && abs((latitude_mdeg / E7 -POINTLAT) < DIFF )){
          avatar.setColorPalette(*cps[3]);
          avatar.setSpeechText("Here!");
          #ifdef USESERVO
            synchronizeAllServosStartAndWaitForAllServosToStop();
            servo_y.easeTo(60);
            servo_x.easeTo(90);
            delay(200);
            servo_y.easeTo(80);
            servo_x.easeTo(95);
          #endif
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
    if (ambientcnt > ambientSendPeriod){
      ambuffer[strlen(ambuffer)-1] = '\0';
      sprintf(&ambuffer[strlen(ambuffer)], "]}\r\n");
      Serial.write(ambuffer,strlen(ambuffer));
      avatar.setMouthOpenRatio(1.0);
      int r = ambient.bulk_send(ambuffer);
      avatar.setMouthOpenRatio(0.0);
      Serial.println(r);
      if (r){
        ambientcnt=0;
        Serial.println("ambient send");
        sprintf(ambuffer, "{\"writeKey\":\"%s\",\"data\":[", writeKey);
        avatar.setSpeechText("SendData");
        return;
      }else{
        Serial.println("Ambient send error");
        // return;
      }
    }else{
      ambientcnt++;
    }
      

    lastTransmittedGGA_ms = millis();
//Push our current GGA sentence to caster
//      ntripClient.print(ggaSentence);
//      ntripClient.print("\r\n");
    ggaTransmitComplete = true;
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

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//We will look for and copy the GGA sentence
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  // nmea.process(incoming);
  //Take the incoming char from the u-blox I2C port and check to see if we should record it or not
  if (incoming == '$' && ggaTransmitComplete == true)
  {
    ggaSentenceStarted = true;
    ggaSentenceSpot = 0;
    ggaSentenceEndSpot = sizeof(ggaSentence);
    ggaSentenceComplete = false;
  }

  if (ggaSentenceStarted == true)
  {
    ggaSentence[ggaSentenceSpot++] = incoming;

    //Make sure we don't go out of bounds
    if (ggaSentenceSpot == sizeof(ggaSentence))
    {
      //Start over
      ggaSentenceStarted = false;
    }
    //Verify this is the GGA setence
    else if (ggaSentenceSpot == 5 && incoming != 'G')
    {
      //Ignore this sentence, start over
      ggaSentenceStarted = false;
    }
    else if (incoming == '*')
    {
      //We're near the end. Keep listening for two more bytes to complete the CRC
      ggaSentenceEndSpot = ggaSentenceSpot + 2;
    }
    else if (ggaSentenceSpot == ggaSentenceEndSpot)
    {
      ggaSentence[ggaSentenceSpot] = '\0'; //Terminate this string
      ggaSentenceComplete = true;
      ggaTransmitComplete = false; //We are ready for transmission

      //Serial.print("GGA Parsed - ");
      //Serial.println(ggaSentence);

      //Start over
      ggaSentenceStarted = false;
    }
  }
}
