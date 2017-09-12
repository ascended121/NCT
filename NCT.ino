/*
NCT
New Cell Tracker

This code is intended to read from the GPS, log to the SD card, and ocassionally send SMS
message with the location data.
*/

/*
 * Includes
 */
// SD includes
#include <SPI.h>
#include <SD.h>
// GPS includes
#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>
// FONA includes
#include "Adafruit_FONA.h"
// other includes
#include <EEPROM.h>

/*
 * Defines
 */
// pin aliases
#define PIN_GPS_RX 0
#define PIN_GPS_TX 1
#define PIN_FONA_INTRPT 2  // RI pin
#define PIN_SD_CS 4
#define PIN_FONA_RST 5 // Key pin
#define PIN_SD_MOSI 11
#define PIN_SD_MISO 12
#define PIN_SD_CLKSO 13
#define PIN_FONA_RX 14
#define PIN_FONA_TX 15
#define PIN_DEBUG_RX 16
#define PIN_DEBUG_TX 17

// program parameters
#define GPSECHO false
#define MAX_AUTH_COMMANDERS 5
#define MAX_SMS_RECIPIENTS 5
#define SMS_BUFF_SIZE 140

// Serial object aliases
// so that the user doesn't have to keep track of which is which
#define debugSerial Serial2
#define fonaSerial Serial3
#define gpsSerial Serial

/* 
 *  Declare and intitialize objects
 */
//SoftwareSerial fonaSerial = SoftwareSerial(PIN_FONA_TX, PIN_FONA_RX);
//SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(PIN_FONA_RST);

// declare and intitialize gps objects
//SoftwareSerial gpsSerial = SoftwareSerial(PIN_GPS_TX, PIN_GPS_RX);
//SoftwareSerial *gpsSerial = &gpsSS;
Adafruit_GPS GPS(&gpsSerial);

// declare and intitialize sd objects
File dataFile;

/* 
 *  Global Variables
 */
// this keeps track of whether we're using the interrupt
// off by default!
// this is a large buffer for replies
char replybuffer[255];
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
uint16_t smsSeqCtr = 0;

/* 
 *  Function prototypes
 */
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


/*
 * Setup
 * This function is responsible for initializing and configuring the GPS modules,
 * the FONA module, and the SD logging
 */
void setup() {

  // Debug serial
  debugSerial.begin(115200);
  debugSerial.println(F("NCT initialization... (may take several seconds)"));

  setupGps();

  setupFona();

  setupSd();

  setupNums();
  
  // Attach an interrupt to the ISR vector
  attachInterrupt(0, smsAvailIsr, LOW);
  
  // enable the GPS reading interrupt
  useInterrupt(true);

}

/*
 * Loop function executes the functionality of the program
 * The GPS is read via an interrupt and so this function just processes the result
 * once its been fully read. The FONA also produces an interrupt when it receives an
 * SMS message and so don't need to be polled.
 */
void loop() {

  // flag indicating if there was new GPS data produced this cycle which has been processed
  boolean parsed_data = false;
  
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
  if (smsAvail){
    
    // iterate through the SMS slots on the FONA
    int slot = 0;     

    // keep track of how many messages have been read
    uint8_t smsRead = 0;
    
    while (smsRead < fona.getNumSMS()) {
        uint16_t smslen;
     
        debugSerial.print(F("\n\rReading SMS #")); 
        debugSerial.println(slot);

        // read the SMS into a buffer
        uint8_t len = fona.readSMS(slot, smsBuffer, SMS_BUFF_SIZE, &smslen); 
        
        // if the length is zero, this is an empty slot and we need to keep reading
        if (len == 0) {
          debugSerial.println(F("[empty slot]"));
          continue;
        }
        // otherwise, we read a message
        else{
          smsRead++;
        }

        // get the phone number which sent the SMS
        if (! fona.getSMSSender(slot, sender, sizeof(sender))) {
          // failed to get the sender?
          sender[0] = 0;
        }

        // process the SMS message we received
        processSmsCmd();
     
        // delete the original msg after it is processed, otherwise, we will fill 
        //  up all the SMS slots and then we won't be able to receive SMS anymore
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
   * An SMS with the payload telemetry will be sent if a command to force an
   * SMS send has been received or if the SMS messages have been enabled and 
   * enough time has ellapsed from the previous message
   */
  if(forceSendSms || sendSms && (millis() - timer > smsPeriod_sec*1000)){
    
    // variable to store battery voltage
    uint16_t fonaBattVoltage = 0;
    
    fona.getBattVoltage(&fonaBattVoltage);

    // note that if you try to make too long of a message, it will be truncated
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "%04d %04d/%02d/%02d %02d:%02d:%02d - Lat: %8.4f deg, Lon: %8.4f deg, Alt: %06d ft, Heading: %5.3f deg, Speed: %5.2f mph, FixQual: %02d, Sats: %02d, BattV: %05d mV, Log: %01d", 
    smsSeqCtr, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude, GPS.angle, GPS.speed, GPS.fixquality, GPS.satellites, fonaBattVoltage, (boolean)dataFile);

    // send the message to all recipients in our table
    for(uint8_t i = 0; i < MAX_SMS_RECIPIENTS; i++){
      fona.sendSMS(smsRecipientsList[i], smsSendBuffer);
    }

    // reset the force flag
    forceSendSms = false;
    smsSeqCtr++;
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

/*
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
*/

/*
 * smsAvailIsr()
 * This method is called when the FONA trips an interrupt indicating it has a 
 * new SMS. All it does is set a flag so that next cycle the SMS will be read
 */
void smsAvailIsr() {
  // just set a flag so that the next loop can read the SMS
  smsAvail = true;
}

/*
 * setupGps()
 * Setup the GPS serial and configure the GPS to output solutions
 */
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

/*
 * setupFona()
 * Setup the GPS and the serial, also configure the FONA to produce an interrupt
 * when it receives an SMS
 */
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

  // do we need to unlock the sim card

  //fonaSerial.print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  // turn on RI pin change on incoming SMS!
  pinMode(PIN_FONA_INTRPT, INPUT);
  digitalWrite(PIN_FONA_INTRPT, HIGH); // turn on pullup on RI
  fona.sendCheckReply(F("AT+CFGRI=1"), F("OK"));
  
  debugSerial.println("FONA Ready");
}

/*
 * setupSd()
 * Setup the SD card for logging and open the logfile
 */
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

/*
 * signal()
 * Interrupt is called once a millisecond, looks for any new GPS data, and stores it
 */
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

/*
 * useInterrupt()
 * Enable the interrupt to read GPS data in the background
 */
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

/*
 * isAuthSender()
 * Function which checks if a given number is in the list of authorized senders
 */
boolean isAuthSender(char *sender){
  for (uint8_t i = 0; i < MAX_AUTH_COMMANDERS; i++){
    if(strcmp(authCommanderList[i],sender)){
      return true;
    }
  }
  return false;
}

/*
 * processSmsCmd()
 * Parses the contents of an SMS message and execute any contained commands
 */
void processSmsCmd(){

  // flag to track if we found a command so that we can throw an error if not
  boolean cmdExed = false;

  // counter to track location that substrings were located
  uint8_t str_loc = 0;

  // cell number extracted
  char numberStr[11];
  
  /* Implement Enable SMS command
   *  Syntax: "enable sms", case insensitive
   *  This command executes by setting a flag so that the main loop doesnt send
   *  an SMS message
   */
  if(strcasestr(smsBuffer,"enable sms") && isAuthSender(sender)){
    sendSms = true;
    cmdExed = true;
  }
  /* Implement Disable SMS command
   *  Syntax: "disable sms", case insensitive
   *  This command executes by setting a flag so that the main loop doesnt send
   *  an SMS message
   */
  if(strcasestr(smsBuffer,"disable sms") && isAuthSender(sender)){
    sendSms = false;
    cmdExed = true;
  }
  /* Implement SMS Frequency command
   *  Syntax: "sms freq <frequency>", case insensitive, frequency is an integer
   *  This command is executed by changing the loop delay variable so that the loop
   *  executes faster.
   */
  //
  if(strcasestr(smsBuffer,"sms freq") && isAuthSender(sender)){
    // Fixme: Get value from message
    smsPeriod_sec = 60;
    cmdExed = true;
  }
  /* Implement Force Send command
   *  Syntax: "force send", case insensitive
   *  This command is implemented by setting a flag such that on the next loop cycle
   *  a message will be sent after reading the sensors
   */
  if(strcasestr(smsBuffer,"force send") && isAuthSender(sender)){
    forceSendSms = true;
    cmdExed = true;
  }
  /* Implement Add Auth Num command
   *  Syntax: "add auth <cell_number>", case insensitive, cell number with dash delimiters
   *  This command adds a new cell number to the list of phone numbers which are permitted to 
   *  command this payload. A response SMS will be sent back to the user to confirm the command.
   */
  if((str_loc = strcasestr(smsBuffer,"add auth")) && isAuthSender(sender)){
    extractPhoneNum(numberStr);
    addAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, numberStr, "auth commanders list");
    cmdExed = true;
  }
  /* Implement Remove Auth Num command
   *  Syntax: "remove auth <cell_number>", case insensitive, cell number with dash delimiters
   */
  if((str_loc = strcasestr(smsBuffer,"remove auth")) && isAuthSender(sender)){
    extractPhoneNum(numberStr);
    removeAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, numberStr, "auth commanders list");
    cmdExed = true;
  }
  /* Implement Add SMS Recipient command
   *  Syntax: "add sms <cell_number>", case insensitive, cell number with dash delimiters
   */
  if((str_loc = strcasestr(smsBuffer,"add sms")) && isAuthSender(sender)){
    extractPhoneNum(numberStr);
    addAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, numberStr, "sms recipients list");
    cmdExed = true;
  }
  /* Implement Remove SMS Recipient command
   *  Syntax: "remove sms <cell_number>", case insensitive, cell number with dash delimiters
   */
  if((str_loc = strcasestr(smsBuffer,"remove sms")) && isAuthSender(sender)){
    extractPhoneNum(numberStr);
    removeAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, numberStr, "sms recipients list");
    cmdExed = true;
  }
  // Send a message back if the SMS didn't contain a valid command so that the user knows
  if(~cmdExed){
    fona.sendSMS(sender, "Unrecognized command");
  }
}

/*
 * extractPhoneNum()
 *  Extracts a phone number of the format XXX-XXX-XXXX from a string
 */
void extractPhoneNum(char *numberStr){
  // cell number sections
  uint16_t firstInt = 0;
  uint16_t secondInt = 0;
  uint16_t thirdInt = 0;
  // strings containing the number sections
  char firstStr[4];
  char secondStr[4];
  char thirdStr[5];

  // extract the number sections from the string
  sscanf(smsBuffer,"%d-%d-%d", firstInt, secondInt, thirdInt);

  // convert the int back to strings
  itoa(firstInt, firstStr, 10);
  itoa(firstInt, secondStr, 10);
  itoa(firstInt, thirdStr, 10);

  // then merge the sections to form the full number
  strcpy(numberStr, firstStr);
  strcat(numberStr, secondStr);
  strcat(numberStr, thirdStr);
}

/*
 * setupNums()
 * Initalizes the SMS recipients list and the authorized commanders lists
 */
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

/*
 * addNumListEntry()
 * Add an entry to the specified list
 */
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

/*
 * clearList()
 * Clear all entries from the given list
 */
boolean clearList(char list[][25], uint8_t list_len){
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    list[i][0] = '\0';
  }
  return true; 
}

/*
 * addAndRespond()
 * Adds an entry to a specified list and send an SMS message back to the user to confirm
 */
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

/*
 * removeAndRespond()
 * Removes an entry from the specified list and notifies the user
 */
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

/*
 * removeNumListEntry()
 * Removes an entry from the specified list. Note, this does not protect against 
 * removing everyone from a list, which could be a problem for the AuthCommand list.
 */
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



