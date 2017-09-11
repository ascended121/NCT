/*
NCT
New Cell Tracker

This code is intended to read from the GPS, log to the SD card, and ocassionally send SMS
message with the location data.
*/

// SD includes
#include <SPI.h>
#include <SD.h>
// GPS includes
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// FONA includes
#include "Adafruit_FONA.h"

#define PIN_DEBUG_RX 0
#define PIN_DEBUG_TX 1
#define PIN_FONA_INTRPT 2
#define PIN_GPS_INTRPT 3
#define PIN_SD_CS 4
#define PIN_FONA_RST 5
#define PIN_GPS_RX 7
#define PIN_GPS_TX 8
#define PIN_FONA_RX 9
#define PIN_FONA_TX 10
#define PIN_SD_MOSI 11
#define PIN_SD_MISO 12
#define PIN_SD_CLKSO 13

#define GPSECHO false

//// Serial object aliases
// so that the user doesn't have to keep track of which is which
#define debugSerial Serial

// this is a large buffer for replies
char replybuffer[255];

// declare and intitialize fona objects
SoftwareSerial fonaSerial = SoftwareSerial(PIN_FONA_TX, PIN_FONA_RX);
//SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(PIN_FONA_RST);

// declare and intitialize gps objects
SoftwareSerial gpsSerial = SoftwareSerial(PIN_GPS_TX, PIN_GPS_RX);
//SoftwareSerial *gpsSerial = &gpsSS;
Adafruit_GPS GPS(&gpsSerial);

// declare and intitialize sd objects
File dataFile;

// Global Variables
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
uint32_t timer = millis();

// Function prototypes
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

void setup() {

  // Debug serial
  debugSerial.begin(115200);
  debugSerial.println(F("FONA SMS caller ID test"));
  debugSerial.println(F("Initializing....(May take 3 seconds)"));

  // GPS serial
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  gpsSerial.begin(115200);
  gpsSerial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // output RMC and GGA strings
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);             // something about antenna state

  // make it slow so its easy to read!
  fonaSerial.begin(4800);
  if (! fona.begin(fonaSerial)) {
    debugSerial.println(F("Couldn't find FONA"));
    while(1);
  }
  debugSerial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    debugSerial.print("SIM card IMEI: ");
    debugSerial.println(imei);
  }

  fonaSerial.print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received
  debugSerial.println("FONA Ready");

  // see if the card is present and can be initialized:
  if (!SD.begin(PIN_SD_CS)) {
    debugSerial.println("Card failed, or not present");
  }
  debugSerial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("datalog.txt", FILE_WRITE);

  useInterrupt(true);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
  
char fonaNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];

void loop() {
  
  char* bufPtr = fonaNotificationBuffer;    //handy buffer pointer

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    debugSerial.print("\nTime: ");
    debugSerial.print(GPS.hour, DEC); debugSerial.print(':');
    debugSerial.print(GPS.minute, DEC); debugSerial.print(':');
    debugSerial.print(GPS.seconds, DEC); debugSerial.print('.');
    debugSerial.println(GPS.milliseconds);
    debugSerial.print("Date: ");
    debugSerial.print(GPS.day, DEC); debugSerial.print('/');
    debugSerial.print(GPS.month, DEC); debugSerial.print("/20");
    debugSerial.println(GPS.year, DEC);
    debugSerial.print("Fix: "); debugSerial.print((int)GPS.fix);
    debugSerial.print(" quality: "); debugSerial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      debugSerial.print("Location: ");
      debugSerial.print(GPS.latitude, 4); debugSerial.print(GPS.lat);
      debugSerial.print(", "); 
      debugSerial.print(GPS.longitude, 4); debugSerial.println(GPS.lon);
      debugSerial.print("Location (in degrees, works with Google Maps): ");
      debugSerial.print(GPS.latitudeDegrees, 4);
      debugSerial.print(", "); 
      debugSerial.println(GPS.longitudeDegrees, 4);
      
      debugSerial.print("Speed (knots): "); debugSerial.println(GPS.speed);
      debugSerial.print("Angle: "); debugSerial.println(GPS.angle);
      debugSerial.print("Altitude: "); debugSerial.println(GPS.altitude);
      debugSerial.print("Satellites: "); debugSerial.println((int)GPS.satellites);
    }
  }
  
  if (fona.available())      //any data available from the FONA?
  {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      debugSerial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      debugSerial.print(F("slot: ")); 
      debugSerial.println(slot);
      
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        debugSerial.println(F("Didn't find SMS message in slot!"));
      }
      debugSerial.print(F("FROM: "));
      debugSerial.println(callerIDbuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (fona.readSMS(slot, smsBuffer, 250, &smslen)) { // pass in buffer and max len!
          debugSerial.println(smsBuffer);
        }

      //Send back an automatic response
      debugSerial.println("Sending reponse...");
      if (!fona.sendSMS(callerIDbuffer, "Hey, I got your text!")) {
        debugSerial.println(F("Failed"));
      } else {
        debugSerial.println(F("Sent!"));
      }
      
      // delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) {
        debugSerial.println(F("OK!"));
      } else {
        debugSerial.print(F("Couldn't delete SMS in slot "));
        debugSerial.println(slot);
        fona.print(F("AT+CMGD=?\r\n"));
      }
    }
  }

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(F("This is a test"));
    dataFile.close();
    // print to the serial port too:
    debugSerial.println(F("This is a test"));
  }
  // if the file isn't open, pop up an error:
  else {
    debugSerial.println(F("error opening datalog.txt"));
  }
}
