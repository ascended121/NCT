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
// other includes
#include <EEPROM.h>

#define PIN_DEBUG_RX 0
#define PIN_DEBUG_TX 1
#define PIN_FONA_INTRPT 2
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
#define MAX_AUTH_COMMANDERS 5
#define MAX_SMS_RECIPIENTS 5
#define SMS_BUFF_SIZE 140

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
uint32_t smsPeriod_sec = 60*5;
char smsBuffer[SMS_BUFF_SIZE];
char smsSendBuffer[SMS_BUFF_SIZE];
char sender[25];
char smsRecipientsList[MAX_SMS_RECIPIENTS][25];
char authCommanderList[MAX_AUTH_COMMANDERS][25];
volatile boolean smsAvail = false;
boolean sendSms = true;
boolean forceSendSms = false;
 
// Function prototypes
void useInterrupt(boolean);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
void setupSd();
void setupGps();
void setupFona();
void processSmsCmd();
void upStr(char *str);
void setupNums();
void addAndRespond(char list[][25], uint8_t list_len, char *add_num, char* list_name);
void removeAndRespond(char list[][25], uint8_t list_len, char *remove_num, char* list_name);
boolean removeNumListEntry(char list[][25], uint8_t list_len, char *remove_num);
boolean addNumListEntry(char list[][25], uint8_t list_len, char *add_num);
boolean clearList(char list[][25], uint8_t list_len);

void setup() {

  // Debug serial
  debugSerial.begin(115200);
  debugSerial.println(F("NCT initialization... (may take several seconds)"));

  setupGps();

  setupFona();

  setupSd();

  setupNums();
  
  // enable the GPS reading interrupt
  useInterrupt(true);

  // Attach an interrupt to the ISR vector
  attachInterrupt(0, smsAvailIsr, LOW);
}

void loop() {

  boolean parsed_data = false;
  uint16_t fonaBattVoltage = 0;
  
  /*
   * First, process the GPS data. The receiving of the actual data is handled by
   * the timer-based interrupt which deposits the string into a buffer. Then, once
   * the whole string has been received, we process it below.
   */
  
  // if a sentence is received (known because we can check the checksum), parse it...
  if (GPS.newNMEAreceived()) {

    // parse the NMEA string and read the variables into the GPS object's properties
    // this also sets the newNMEAreceived() flag to false
    parsed_data = GPS.parse(GPS.lastNMEA()); 
  }

  /*
   * The interrupt routine will record if an SMS has been received, and the
   * SMS will be read and processed here
   */
  fona.getBattVoltage(&fonaBattVoltage);
  if (smsAvail){
    int slot = 0;            //this will be the slot number of the SMS
    uint8_t smsRead = 0;
    while (smsRead < fona.getNumSMS()) {
        uint16_t smslen;
     
        debugSerial.print(F("\n\rReading SMS #")); 
        debugSerial.println(slot);
     
        uint8_t len = fona.readSMS(slot, smsBuffer, SMS_BUFF_SIZE, &smslen); // pass in buffer and max len!
        // if the length is zero, its a special case where the index number is higher
        // so increase the max we'll look at!
        if (len == 0) {
          debugSerial.println(F("[empty slot]"));
          continue;
        }
        // otherwise, we read a message
        else{
          smsRead++;
        }
        if (! fona.getSMSSender(slot, sender, sizeof(sender))) {
         // failed to get the sender?
         sender[0] = 0;
        }

        processSmsCmd();
     
        // delete the original msg after it is processed, otherwise, we will fill 
        //  up all the slots and then we won't be able to receive SMS anymore
        if (fona.deleteSMS(slot)) {
          debugSerial.println(F("OK!"));
        } else {
          debugSerial.print(F("Couldn't delete SMS in slot "));
          debugSerial.println(slot);
        }
        slot++;
    }
  }

  /*
   * Send a telemetry message
   */
  if(forceSendSms || sendSms && (millis() - timer > smsPeriod_sec*1000)){

    // note that if you try to make too long of a message, it will be truncated
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "%d:%d:%d - Lat: %f deg, Lon: %f deg, Alt: %f ft, Heading: %f deg, Speed: %f mph, FixQual: %d, Sats: %d, BattV: %f mV", 
    GPS.hour, GPS.minute, GPS.seconds, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude, GPS.angle, GPS.speed, GPS.fixquality, GPS.satellites, fonaBattVoltage);

    // send the message to all recipients
    for(uint8_t i = 0; i < MAX_SMS_RECIPIENTS; i++){
      fona.sendSMS(smsRecipientsList[i], smsSendBuffer);
    }

    // reset the force flag
    forceSendSms = false;
  }

  /*
   * Log data to SD card
   */
  if (dataFile) {
    if (parsed_data){  
     // log the data using the same message we just sent via SMS
     dataFile.println(smsSendBuffer); 
    }
  }
  // if the file isn't open, pop up an error:
  else {
    debugSerial.println(F("error opening datalog.txt"));
  }
}


void print_GPS_data(){
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

void smsAvailIsr() {
  // just set a flag so that the next loop can read the SMS
  smsAvail = true;
}

void setupGps(){
  // GPS serial
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  gpsSerial.begin(115200);
  gpsSerial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // output RMC and GGA strings
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);             // something about antenna state
}


void setupFona(){
  // make it slow so its easy to read!
  fonaSerial.begin(4800);
  if (! fona.begin(fonaSerial)) {
    debugSerial.println(F("Couldn't find FONA"));
  }
  debugSerial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    debugSerial.print("SIM card IMEI: ");
    debugSerial.println(imei);
  }

  //fonaSerial.print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  // turn on RI pin change on incoming SMS!
  pinMode(PIN_FONA_INTRPT, INPUT);
  digitalWrite(PIN_FONA_INTRPT, HIGH); // turn on pullup on RI
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));
  
  debugSerial.println("FONA Ready");
}

void setupSd(){
  // see if the card is present and can be initialized:
  if (!SD.begin(PIN_SD_CS)) {
    debugSerial.println("Card failed, or not present");
  }
  debugSerial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("datalog.txt", FILE_WRITE);
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

boolean isAuthSender(char *sender){
  for (uint8_t i = 0; i < MAX_AUTH_COMMANDERS; i++){
    if(strcmp(authCommanderList[i],sender)){
      return true;
    }
  }
  return false;
}

void processSmsCmd(){
  //smsBuffer[250];
  //sender[25];
  // Implement Enable SMS command
  if(strcasestr(smsBuffer,"enable") && strcasestr(smsBuffer,"sms") && isAuthSender(sender)){
    sendSms = true;
  }
  // Implement Disable SMS command
  if(strcasestr(smsBuffer,"disable") && strcasestr(smsBuffer,"sms") && isAuthSender(sender)){
    sendSms = false;
  }
  // Implement SMS Frequency command
  if(strcasestr(smsBuffer,"sms") && strcasestr(smsBuffer,"freq") && isAuthSender(sender)){
    // Fixme: Get value from message
    smsPeriod_sec = 60;
  }
  // Implement Force Send command
  if(strcasestr(smsBuffer,"remove") && strcasestr(smsBuffer,"sms") && isAuthSender(sender)){
    forceSendSms = true;
  }
  // Implement Add Auth Num command
  if(strcasestr(smsBuffer,"add") && strcasestr(smsBuffer,"auth") && isAuthSender(sender)){
    addAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, sender, "auth commanders list");
  }
  // Implement Remove Auth Num command
  if(strcasestr(smsBuffer,"remove") && strcasestr(smsBuffer,"auth") && isAuthSender(sender)){
    removeAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, sender, "auth commanders list");
  }
  // Implement Add SMS Recipient command
  if(strcasestr(smsBuffer,"add") && strcasestr(smsBuffer,"sms") && isAuthSender(sender)){
    addAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, sender, "sms recipients list");
  }
  // Implement Remove SMS Recipient command
  if(strcasestr(smsBuffer,"remove") && strcasestr(smsBuffer,"sms") && isAuthSender(sender)){
    removeAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, sender, "sms recipients list");
  }
}

void setupNums(){
  //EEPROM.length()
  //value = EEPROM.read(address);
  
  // populate the recipients list
  clearList(smsRecipientsList, MAX_SMS_RECIPIENTS);
  addNumListEntry(smsRecipientsList, MAX_SMS_RECIPIENTS, "6033218095");

  // populate the authorized commanders list
  clearList(authCommanderList, MAX_AUTH_COMMANDERS);
  addNumListEntry(authCommanderList, MAX_AUTH_COMMANDERS, "6033218095");

}

boolean addNumListEntry(char list[][25], uint8_t list_len, char *add_num){
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    // check if this entry is empty
    if(strcmp(list[i],'\0')){
      // remove them (set it to nul string) and return success
      strcpy(list[i],add_num);
      return true;
    }
  }
  // if we made it here there wasn't any free space in the list, return failure
  return false; 
}

boolean clearList(char list[][25], uint8_t list_len){
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    list[i][0] = '\0';
  }
  return true; 
}

void addAndRespond(char list[][25], uint8_t list_len, char *add_num, char* list_name){
  if(addNumListEntry(list, list_len, add_num)){
    // send SMS indicating success to commander and the new authorized commander
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully added %s to %s", add_num, list_name);
    fona.sendSMS(sender, smsSendBuffer);
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully added you to %s", list_name);
    fona.sendSMS(sender, smsSendBuffer);
  }
  else{
    // send SMS indicating failure
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Failed to add %s to %s", add_num, list_name);
    fona.sendSMS(sender, smsSendBuffer);
  }
}

void removeAndRespond(char list[][25], uint8_t list_len, char *remove_num, char* list_name){
  if(removeNumListEntry(list, list_len, remove_num)){
    // send SMS indicating success
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully removed %s from %s", remove_num, list_name);
    fona.sendSMS(sender, smsSendBuffer);
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully removed you from %s", list_name);
    fona.sendSMS(sender, smsSendBuffer);
  }
  else{
    // send SMS indicating failure
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Failed to remove %s from %s", remove_num, list_name);
    fona.sendSMS(sender, smsSendBuffer);
  }
}

boolean removeNumListEntry(char list[][25], uint8_t list_len, char *remove_num){
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    // check if its the one we're supposed to remove
    if(strcmp(list[i],remove_num)){
      // remove them (set it to nul string) and return success
      list[i][0] = '\0';
      return true;
    }
  }
  // if we made it here we didn't find them in the list, return failure
  return false;
}

