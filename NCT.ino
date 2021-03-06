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
/*
 * Note: This IS NOT the default SD library that comes with the Arduino IDE. Because this shield is
 * being used with a Mega a different library is needed that supports software SPI. See the compatibility 
 * section of the adafruit tutorial for this shield for details.
 */

// GPS includes
#include <Adafruit_GPS.h>

// FONA includes
#include "Adafruit_FONA.h"
/* Note: this library contains a debug printf that's hardcoded to a specific serial, so
 *  if you see unexpected output on a serial, it might be this. To enable/disable the debug 
 *  output change the file Arduino\libraries\Adafruit_FONA_Library\includes\FONA_config.h
 *  To change which serial it prints to, change 
 *  Arduino\libraries\Adafruit_FONA_Library\includes\platform\FONAPlatStd.h
 */
 
// other includes
#include <EEPROM.h>
#include "default_phone_numbers.h"
// NOTE: THESE MUST BE UDPDATED BEFORE DEPLOYING THE CODE

/*
 * Defines
 */
// pin aliases
#define PIN_DEBUG_RX     0 // Mega TX0
#define PIN_DEBUG_TX     1 // Mega RX0
#define PIN_FONA_RST     2 // FONA Rst pin
#define PIN_FONA_INTRPT  3 // FONA RI pin
#define PIN_FONA_KEY     5 // FONA Key pin
#define PIN_GPS_RX       8 // Mega RX3
#define PIN_GPS_TX       7 // Mega RX3
#define PIN_SD_CS        10 
#define PIN_SD_MOSI      11
#define PIN_SD_MISO      12
#define PIN_SD_CLKSO     13
#define PIN_FONA_RX      14 // Mega TX2
#define PIN_FONA_TX      15 // Mega RX2

// constants
#define MSEC_PER_SEC       1000
#define MSEC_PER_MIN       60*MSEC_PER_SEC
#define PHONE_NUM_MAX_LEN  25

// FONA Network status
#define FONA_NET_NOTREG       0
#define FONA_NET_REGISTERED   1
#define FONA_NET_SEARCHING    2
#define FONA_NET_DENIED       3
#define FONA_NET_UNKNOWN      4
#define FONA_NET_ROAMING      5 

// program parameters
#define SMS_FREQ_MIN         10
#define SMS_FREQ_MAX       3600
#define MAX_AUTH_COMMANDERS   5
#define MAX_SMS_RECIPIENTS    5
#define MIN_AUTH_COMMANDERS   2
#define SMS_BUFF_SIZE       140
#define LOG_BUFF_SIZE       250
#define LOGFILE_NAME    "datalog.csv"
#define EEPROM_RECIPIENTS_START_ADDR  0
#define EEPROM_COMMANDERS_START_ADDR  (EEPROM_RECIPIENTS_START_ADDR+PHONE_NUM_MAX_LEN*MAX_SMS_RECIPIENTS)
#define EEPROM_LAST_SMS_TIME_ADDR     (EEPROM_COMMANDERS_START_ADDR+PHONE_NUM_MAX_LEN*MAX_AUTH_COMMANDERS)

// debugging
#define GPSECHO false   

// Serial object aliases
// so that the user doesn't have to keep track of which is which
#define debugSerial Serial
#define fonaSerial  Serial2
#define gpsSerial   Serial3

/* 
 *  Declare and intialize objects
 */
Adafruit_FONA fona = Adafruit_FONA(PIN_FONA_RST);

// declare and intitialize gps objects
Adafruit_GPS GPS(&gpsSerial);

// declare and intitialize sd objects
File dataFile;

/* 
 *  Global Variables
 */
boolean   usingGpsInterrupt   = false;          // this will be enabled once the GPS has been configured
uint16_t  smsSeqCtr         = 0;                // SMS sequence counter
uint16_t  logSeqCtr         = 0;                // Log file entry sequence counter
uint8_t   netStatus         = FONA_NET_UNKNOWN; // status of connection to cell network
uint16_t  fonaBattVoltage   = 0;                // battery voltage level
uint8_t   fonaRssi          = 0;                // signal strength
uint16_t  rcvdSmsCtr        = 0;                // counter tracking number of received SMS

// lists
char      smsRecipientsList[MAX_SMS_RECIPIENTS][PHONE_NUM_MAX_LEN];   // list of phone numbers to send SMS's to 
char      authCommanderList[MAX_AUTH_COMMANDERS][PHONE_NUM_MAX_LEN];  // list of phone numbers to accept commands from

// timers
uint32_t  smsPeriod_msec       = 5*MSEC_PER_MIN;   // default to 5min between SMS
uint32_t  fonaStatusPeriod_msec= 10*MSEC_PER_SEC;  // default to 10sec between fona status checks
uint32_t  smsTimer;                                // this will be initialized in setup from eeprom memory
uint32_t  fonaStatusTimer      = 0;                // initalize to zero to make this trigger on first loop

// buffers
char      smsRcvBuffer[SMS_BUFF_SIZE];          // buffer to store received SMS's
char      smsSendBuffer[SMS_BUFF_SIZE];         // buffer in which to create outgoing SMS's
char      logBuffer[LOG_BUFF_SIZE];             // buffer in which to create string to be logged
char      sender[25];                           // buffer to store phone number of SMS sender

// program control flags
volatile boolean smsAvail   = false;            // flag, set by the SMS interrupt, to read an SMS
boolean   smsEnableFlag     = false;            // flag which enables the sending of telemetry SMS's to the recipients
boolean   forceSendSms      = false;            // flag which forces the sending of a telemetry SMS to the recipients
boolean   missedSmsSend     = false;            // flag indicating if an SMS sending opportunity was missed due to lack of network
 
/* 
 *  Function prototypes
 */
// setup 
void setupSd();
void setupGps();
void setupFona();
void setupLists();

// SMS I/O
boolean sendSmsIfNetwork(char *smsaddr, char *smsmsg);

// Commands implementation
void processSmsCmd();
void addAndRespond(char list[][25], uint8_t list_len, char *add_num, char *list_name, uint16_t start_eeprom_addr);
void removeAndRespond(char list[][25], uint8_t list_len, char *remove_num, char *list_name, uint16_t start_eeprom_addr);
void respondWithList(char list[][25], uint8_t list_len, char *list_name);

// command helper
boolean removeNumListEntry(char list[][25], uint8_t list_len, char *remove_num, uint16_t start_eeprom_addr);
boolean addNumListEntry(char list[][25], uint8_t list_len, char *add_num, uint16_t start_eeprom_addr);
boolean clearList(char list[][25], uint8_t list_len, uint16_t start_eeprom_addr);
uint8_t countListEntries(char list[][25], uint8_t list_len);

// utility
void useInterrupt(boolean);
boolean haveNetwork();

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

  // enable the GPS reading interrupt
  useInterrupt(true);

  setupFona();

  setupSd();

  setupLists();

  // set smsTimer from last send time in EEPROM
  // This prevent rolling blackouts from spamming SMS's
  EEPROM.get(EEPROM_LAST_SMS_TIME_ADDR, smsTimer);
  
  // Attach an interrupt to the ISR vector to trigger checking for SMSs
  attachInterrupt(digitalPinToInterrupt(PIN_FONA_INTRPT), smsAvailIsr, LOW);
  
}

/*
 * Loop function executes the functionality of the program
 * The GPS is read via an interrupt and so this function just processes the result
 * once its been fully read. The FONA also produces an interrupt when it receives an
 * SMS message and so don't need to be polled.
 */
void loop() {

  // flag indicating if there was new GPS data produced this cycle which has been processed
  // initalize to false, will be set later
  boolean parsedGpsData = false;

  /*
   * Since the baud to the FONA is so slow, don't want to be requesting data we dont have to, 
   * so only get battery voltage and network status when we need it
   */
  if(millis() - fonaStatusTimer > fonaStatusPeriod_msec){
    // update the timer
    fonaStatusTimer = millis();

    // get the data
    fona.getBattVoltage(&fonaBattVoltage);
    netStatus = fona.getNetworkStatus();
    fonaRssi = fona.getRSSI();
  }
  
  /*
   * First, process the GPS data. The receiving of the actual data is handled by
   * the timer-based interrupt which deposits the string into a buffer. Then, once
   * the whole string has been received, we process it below.
   */
  // if a new sentence is received (ie, if the checksum validates), parse it...
  if (GPS.newNMEAreceived()) {

    // parse the NMEA string and read the variables into the GPS object's properties
    // this also sets the newNMEAreceived() flag to false
    parsedGpsData = GPS.parse(GPS.lastNMEA()); 
    debugSerial.println("Parsed GPS data!");

    // now we have new data to log/send out via SMS
  }

  /*
   * The interrupt routine will record if an SMS has been received by setting smsAvail
   * to true. This is the logic to read and process the SMS
   */
  if (smsAvail){

    // determine number of SMS that need to be read
    uint8_t totalSms = fona.getNumSMS();

    debugSerial.print("Reading ");
    debugSerial.print(totalSms);
    debugSerial.println(" SMS");

    // reset the flag
    smsAvail = false;

    // Initalize a variable to keep track of which slot is being read
    // Note: Apparently the module starts numbering SMS at 1
    int slot = 1;     

    // because slots can be empty, keep track of how many messages have been read
    uint8_t numSmsRead = 0;

    // iterate through the SMS slots on the FONA
    while (numSmsRead < fona.getNumSMS()) {
     
        debugSerial.print(F("\n\rReading SMS #")); 
        debugSerial.println(slot);

        // this variable isn't actually used, but we need a buffer to read the 
        // value into
        uint16_t smslen;

        // read the SMS into a buffer
        uint8_t len = fona.readSMS(slot, smsRcvBuffer, SMS_BUFF_SIZE, &smslen); 
        
        // if the length is zero, this is an empty slot and we need to keep reading
        if (len == 0) {
          debugSerial.println(F("[empty slot]"));
        }
        // otherwise, we read a message
        else{

          // record that a message has been read
          numSmsRead++;
          rcvdSmsCtr++;

          // get the phone number which sent the SMS
          if (! fona.getSMSSender(slot, sender, sizeof(sender))) {
            // failed to get the sender?
            sender[0] = 0;
          }
  
          // process the SMS message we received
          processSmsCmd();
       
          // delete the original msg after it is processed, otherwise, we will fill 
          //  up all the SMS slots and then we won't be able to receive SMS anymore
          if (!fona.deleteSMS(slot)) {
            debugSerial.print(F("Couldn't delete SMS in slot "));
            debugSerial.println(slot);
          }
        } //if (len == 0)

        // iterate onto the next slot
        slot++;
        
    } // while (numSmsRead < fona.getNumSMS())
  } // if (smsAvail)
  
  /*
   * An SMS with the payload telemetry will be sent if a command to force an
   * SMS send has been received or if the SMS messages have been enabled and 
   * enough time has ellapsed from the previous message
   */
  boolean sendSms = forceSendSms || (smsEnableFlag && (millis() - smsTimer > smsPeriod_msec));

  /*
   * Send a telemetry message
   * If we need to send a message and we have network, create and send it
   */
  if(sendSms || missedSmsSend){
    if(haveNetwork()){
      // update timer
      smsTimer = millis();
  
      /*
       * Create the SMS to send
       * note that if you try to make too long of a message, it will be truncated
       */
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "%04d %04d/%02d/%02d %02d:%02d:%02d-Lat:%8.4fdeg,Lon:%8.4fdeg,Alt:%06dft,Head:%5.3fdeg,Spd:%5.2fmph,FxQual:%02d,Sat:%02d,Batt:%05dmV,Log:%01d,RSSI:%03d", 
      smsSeqCtr, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.latitudeDegrees, 
      GPS.longitudeDegrees, GPS.altitude, GPS.angle, GPS.speed, GPS.fixquality, GPS.satellites, fonaBattVoltage, 
      (boolean)dataFile, fonaRssi);
      debugSerial.print(F("SMS: "));
      debugSerial.println(smsSendBuffer);
  
      // send the message to all recipients in our table
      for(uint8_t i = 0; i < MAX_SMS_RECIPIENTS; i++){
        fona.sendSMS(smsRecipientsList[i], smsSendBuffer);
        // write time of sending to eeprom
        EEPROM.puts(EEPROM_LAST_SMS_TIME_ADDR, smsTimer);
      }
  
      // reset the force flag
      forceSendSms = false;
      missedSmsSend = false;
      smsSeqCtr++;
    }
    else{
        // set flag indicating a send was missed
        missedSmsSend = true;
    }

  }

  /*
   * Log data to SD card
   */
  if (parsedGpsData){  

    // note that if you try to make too long of a message, it will be truncated
    snprintf(logBuffer, LOG_BUFF_SIZE, "%04d,%02d,%02d,%02d,%02d,%02d,%02d,%8.4f,%8.4f,%06d,%5.3f,%5.2f,%02d,%02d,%05d,%01d,%05d", 
    logSeqCtr, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.latitudeDegrees, 
    GPS.longitudeDegrees, GPS.altitude, GPS.angle, GPS.speed, GPS.fixquality, GPS.satellites, fonaBattVoltage, 
    (boolean)dataFile, fonaRssi, netStatus, rcvdSmsCtr);
    debugSerial.print(F("Log: "));
    debugSerial.println(logBuffer);
    
    if (dataFile) {
      // log the data using the same message we just sent via SMS
      dataFile.println(logBuffer); 
      dataFile.flush();

      logSeqCtr++;
    }
    // if the file isn't open, pop up an error:
    else {
      debugSerial.println(F("error opening datalog.txt"));
    }
  }
}

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
 * Returns if we have network (ie, if we can send an SMS)
 */
boolean haveNetwork(){
  return netStatus == FONA_NET_REGISTERED || netStatus == FONA_NET_ROAMING;
}

/*
 * sendSmsIfNetwork()
 * Attempt to send an SMS if we have network
 */
boolean sendSmsIfNetwork(char *smsaddr, char *smsmsg){
  if(haveNetwork()){
    fona.sendSMS(smsaddr,smsmsg);
    return true;
  }
  return false;
}

/*
 * setupGps()
 * Setup the GPS serial and configure the GPS to output solutions
 */
void setupGps(){
  
  // GPS serial
  gpsSerial.begin(115200);
  /*
   * Note: This should be a higher baud rate than the 9600 baud the GPS is using
   * so that the interrupt has time to catch all of the characters.
   */
  debugSerial.println("Begin initalizing GPS...");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // configure the GPS
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);   // 1Hz fix generation rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1Hz fix output rate
  GPS.sendCommand(PGCMD_NOANTENNA);             // turn off output of antenna status packet

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // output RMC and GGA strings
  // Note: this has to be towards the end otherwise it doesn't have any effect

  debugSerial.println("End GPS initalization...");
}

/*
 * setupFona()
 * Setup the GPS and the serial, also configure the FONA to produce an interrupt
 * when it receives an SMS
 */
void setupFona(){
  
  debugSerial.println("Begin FONA initalization...");

  // bring the key pin low to turn on the module
  debugSerial.println("Turning on FONA...");
  pinMode(PIN_FONA_KEY, OUTPUT);
  digitalWrite(PIN_FONA_KEY, LOW);
  delay(2000);
  digitalWrite(PIN_FONA_KEY, HIGH);
  
  // make it slow so its easy to read!
  fonaSerial.begin(4800);
  if (! fona.begin(fonaSerial)) {
    debugSerial.println(F("Couldn't find FONA"));
  }
  else{
    debugSerial.println(F("FONA is OK"));
  }

  delay(10);

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  //if (imeiLen > 0) {
    debugSerial.print("SIM card IMEI: ");
    debugSerial.println(imei);
  //}

  // do we need to unlock the sim card?

  // turn on RI pin change on incoming SMS!
  pinMode(PIN_FONA_INTRPT, INPUT);
  digitalWrite(PIN_FONA_INTRPT, HIGH); // turn on pullup on RI
  fona.setSMSInterrupt(1);

  //fona.sendSMS("6033218095", "Initialized!");

  delay(1000);
  /*
   * Clear the stored SMS's so that we don't fill up
   */
  uint8_t totalSms = fona.getNumSMS();

  if(totalSms > 0){
    debugSerial.print("Deleting ");
    debugSerial.print(totalSms);
    debugSerial.println(" stored SMS...");
  
    uint8_t numSmsDel = 0;
    // Note: Apparently the module starts numbering SMS at 1
    uint8_t slot = 1;
      
    while (numSmsDel < totalSms && slot < 30) {
     
        debugSerial.print(F("Deleting SMS #"));
        debugSerial.print(numSmsDel);
        debugSerial.print(" from slot"); 
        debugSerial.println(slot);
  
        if(fona.deleteSMS(slot)){
          numSmsDel++;
        }
        else{
          debugSerial.println("Error deleting SMS");
        }
      slot++;
    }
  }

  
  
  debugSerial.println("End FONA initalization...");

}

/*
 * setupSd()
 * Setup the SD card for logging and open the logfile
 */
void setupSd(){
  
  debugSerial.println("Begin SD initalization...");

  // chipselect needs to be output so that we can select the SD card on the SPI bus
  pinMode(PIN_SD_CS, OUTPUT);

  /*
   * Determine if the card is present and can be initialized:
   * See note above by the SD card include. This is NOT the normal SD card initialization...
   * its the initialization for the SD card as described in the Adafruit tutorial (compatibility 
   * section) for this shield since they use software SPI to talk to the SD card on this shield.
   */
  if (!SD.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLKSO)) {
    debugSerial.println("Card failed, or not present");
  }
  debugSerial.println("card initialized.");

  // flag indicating if the datalog is a new file
  boolean datalogExists = SD.exists(LOGFILE_NAME);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open(LOGFILE_NAME, FILE_WRITE);

  if(!dataFile){
    debugSerial.println("Could not open file!");
  }

  // if the datalog didn't exist till we opened it, we need to write a header to it
  if(datalogExists){
    debugSerial.println("New datalog, writing header...");
    dataFile.println("SeqCnt,Year,Month,Day,Hour,Min,Sec,Latitude[deg],Longitude[deg],Altitude[ft],Heading[deg],Speed[mph],GPSFixQuality, NumSattelites, BatteryV[mV],LogFileStatus,RSSI");
  }  
  debugSerial.println("End SD initalization...");

}

/*
 * signal()
 * Interrupt is called once a millisecond, looks for any new GPS data, and stores it
 */
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//#ifdef UDR1
  if (GPSECHO)
    debugSerial.print(c);
    //if (c) UDR1 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
//#endif
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
    usingGpsInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingGpsInterrupt = false;
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
 * Parses the contents of an SMS message and execute any contained commands. Only
 * one command will be executed per SMS received.
 */
void processSmsCmd(){

  // determine if sender is authorized, if not, don't bother processing the command
  if(!isAuthSender(sender)){
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Unauthorized sender");
    sendSmsIfNetwork(sender, smsSendBuffer);
    return;
  }

  // counter to track location that substrings were located
  uint8_t str_loc = 0;

  // cell number extracted
  char numberStr[11];
  
  /* Implement Enable SMS command
   *  Syntax: "enable sms", case insensitive
   *  This command executes by setting a flag so that the main loop doesnt send
   *  an SMS message
   */
  if(strcasestr(smsRcvBuffer,"enable sms")){
    smsEnableFlag = true;
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "SMS enabled");
    sendSmsIfNetwork(sender, smsSendBuffer);
  }
  /* Implement Disable SMS command
   *  Syntax: "disable sms", case insensitive
   *  This command executes by setting a flag so that the main loop doesnt send
   *  an SMS message
   */
  if(strcasestr(smsRcvBuffer,"disable sms")){
    smsEnableFlag = false;
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "SMS disabled");
    sendSmsIfNetwork(sender, smsSendBuffer);
  }
  /* Implement SMS Frequency command
   *  Syntax: "sms freq <frequency>", case insensitive, frequency is an integer 
   *  between 10 and 600
   *  This command is executed by changing the loop delay variable so that the loop
   *  executes faster.
   */
  //
  else if(strcasestr(smsRcvBuffer,"sms freq")){
    uint16_t period = 0;
    sscanf(smsRcvBuffer,"%d", period);
    if(period > SMS_FREQ_MIN && period < SMS_FREQ_MAX){
      smsPeriod_msec = period;
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Set period value to %d", period);
      sendSmsIfNetwork(sender, smsSendBuffer);
    }
    else{
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Invalid period value: %d", period);
      sendSmsIfNetwork(sender, smsSendBuffer);
    }
  }
  /* Implement Force Send command
   *  Syntax: "force send", case insensitive
   *  This command is implemented by setting a flag such that on the next loop cycle
   *  a message will be sent after reading the sensors
   */
  else if(strcasestr(smsRcvBuffer,"force send")){
    forceSendSms = true;
  }
  /* Implement Add Auth Num command
   *  Syntax: "add auth <cell_number>", case insensitive, cell number with dash delimiters
   *  This command adds a new cell number to the list of phone numbers which are permitted to 
   *  command this payload. A response SMS will be sent back to the user to confirm the command.
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"add auth"))){
    extractPhoneNum(numberStr);
    addAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, numberStr, "auth commanders list", EEPROM_COMMANDERS_START_ADDR);
  }
  /* Implement Remove Auth Num command
   *  Syntax: "remove auth <cell_number>", case insensitive, cell number with dash delimiters
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"remove auth"))){
    extractPhoneNum(numberStr);
    removeAndRespond(authCommanderList, MAX_AUTH_COMMANDERS, numberStr, "auth commanders list", EEPROM_COMMANDERS_START_ADDR);
  }
  /* Implement List Auth Commanders command
   *  Syntax: "list auth", case insensitive
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"list auth"))){
    respondWithList(authCommanderList, MAX_AUTH_COMMANDERS, "auth commanders list");
  }
  /* Implement Add SMS Recipient command
   *  Syntax: "add sms <cell_number>", case insensitive, cell number with dash delimiters
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"add sms"))){
    extractPhoneNum(numberStr);
    addAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, numberStr, "sms recipients list", EEPROM_RECIPIENTS_START_ADDR);
  }
  /* Implement Remove SMS Recipient command
   *  Syntax: "remove sms <cell_number>", case insensitive, cell number with dash delimiters
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"remove sms"))){
    extractPhoneNum(numberStr);
    removeAndRespond(smsRecipientsList, MAX_SMS_RECIPIENTS, numberStr, "sms recipients list", EEPROM_RECIPIENTS_START_ADDR);
  }
  /* Implement List SMS Recipient command
   *  Syntax: "list sms", case insensitive
   */
  else if((str_loc = strcasestr(smsRcvBuffer,"list sms"))){
    respondWithList(smsRecipientsList, MAX_SMS_RECIPIENTS, "sms recipients list");
  }
  // Send a message back if the SMS didn't contain a valid command so that the user knows
  else{
    sendSmsIfNetwork(sender, "Unrecognized command");
  }
}

/*
 * extractPhoneNum()
 *  Extracts a phone number of the format XXXXXXXXXX from a string
 */
void extractPhoneNum(char *numberStr){

  uint8_t start_loc = 0;
  // loop through until we find a digit, start_loc now has that position
  while(!isdigit(smsRcvBuffer[++start_loc]));

  uint8_t end_loc = start_loc;
  // loop through until we find something other than a digit, end_loc now has that position
  while(isdigit(smsRcvBuffer[end_loc++]));

  // copy the portion of the string between start_loc and end_loc into the response string
  strncpy( numberStr, smsRcvBuffer+start_loc, end_loc-start_loc );

}

/*
 * setupNums()
 * Initalizes the SMS recipients list and the authorized commanders lists
 */
void setupLists(){
  // FIXME: should read the values from last run from eeprom on startup
  //EEPROM.length()
  /*

   char c;
   uint8_t char_pos = 0;
   for(uint8_t i = 0; i < MAX_SMS_RECIPIENTS; i++){
     c = EEPROM.read(EEPROM_RECIPIENTS_START_ADDR + PHONE_NUM_MAX_LEN*i + char_pos);
     while( char_pos < PHONE_NUM_MAX_LEN && c != '\0'){
       smsRecipientsList[i][char_pos] = c;
       char_pos++;
     }
   }
   for(uint8_t i = 0; i < MAX_AUTH_COMMANDERS; i++){
     c = EEPROM.read(EEPROM_COMMANDERS_START_ADDR + PHONE_NUM_MAX_LEN*i + char_pos);
     while( char_pos < PHONE_NUM_MAX_LEN && c != '\0'){
       authCommanderList[i][ii] = c;
       char_pos++;
     }
   }

  */
  
  // populate the recipients listr
  clearList(smsRecipientsList, MAX_SMS_RECIPIENTS, EEPROM_RECIPIENTS_START_ADDR);
  addNumListEntry(smsRecipientsList, MAX_SMS_RECIPIENTS, DEFAULT_RECIPIENT_NUMBER_1, EEPROM_RECIPIENTS_START_ADDR);
  addNumListEntry(smsRecipientsList, MAX_SMS_RECIPIENTS, DEFAULT_RECIPIENT_NUMBER_2, EEPROM_RECIPIENTS_START_ADDR);

  // populate the authorized commanders list
  clearList(authCommanderList, MAX_AUTH_COMMANDERS, EEPROM_COMMANDERS_START_ADDR);
  addNumListEntry(authCommanderList, MAX_AUTH_COMMANDERS, DEFAULT_COMMANDER_NUMBER_1, EEPROM_COMMANDERS_START_ADDR);
  addNumListEntry(authCommanderList, MAX_AUTH_COMMANDERS, DEFAULT_COMMANDER_NUMBER_2, EEPROM_COMMANDERS_START_ADDR);

}

/*
 * addNumListEntry()
 * Add an entry to the specified list
 */
boolean addNumListEntry(char list[][25], uint8_t list_len, char *add_num, uint16_t start_eeprom_addr){
  // FIXME: should write values to eeprom
  
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    // check if this entry is empty
    if(list[i][0] == '\0'){
      // remove them (set it to nul string) and return success
      strcpy(list[i],add_num);
      debugSerial.print("Added to list");

      /*
       * write the number to eeprom to preserve it
      uint8_t char_pos = 0;
      uint16_t eeprom_addr = start_eeprom_addr + PHONE_NUM_MAX_LEN*i;

      while( add_num[char_pos] != '\0'){
       EEPROM.update(eeprom_addr + char_pos, add_num[char_pos]);
       char_pos++;
      }
      // add the terminating character that we skipped above
      EEPROM.update(eeprom_addr + char_pos, '\0');
      */
      
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
boolean clearList(char list[][25], uint8_t list_len, uint16_t start_eeprom_addr){
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    list[i][0] = '\0';
    //EEPROM.update(start_eeprom_addr + PHONE_NUM_MAX_LEN*i, '\0');
  }
  return true; 
}

/*
 * addAndRespond()
 * Adds an entry to a specified list and send an SMS message back to the user to confirm
 */
void addAndRespond(char list[][25], uint8_t list_len, char *add_num, char* list_name, uint16_t start_eeprom_addr){
  if(addNumListEntry(list, list_len, add_num, start_eeprom_addr)){
    // send SMS indicating success to commander and the new authorized commander
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully added %s to %s", add_num, list_name);
    //sendSmsIfNetwork(sender, smsSendBuffer);
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully added you to %s", list_name);
    //sendSmsIfNetwork(sender, smsSendBuffer);
  }
  else{
    // send SMS indicating failure
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Failed to add %s to %s, list full", add_num, list_name);
    sendSmsIfNetwork(sender, smsSendBuffer);
  }
}

/*
 * removeAndRespond()
 * Removes an entry from the specified list and notifies the user
 */
void removeAndRespond(char list[][25], uint8_t list_len, char *remove_num, char* list_name, uint16_t eeprom_start_addr){

  // don't remove the last entry from a list
  if(countListEntries(list, list_len) <= MIN_AUTH_COMMANDERS){
    snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Failed, cannot remove last entry in list");
    sendSmsIfNetwork(sender, smsSendBuffer);
  }
  else{
    if(removeNumListEntry(list, list_len, remove_num, eeprom_start_addr)){
      // send SMS indicating success
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully removed %s from %s", remove_num, list_name);
      sendSmsIfNetwork(sender, smsSendBuffer);
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Successfully removed you from %s", list_name);
      sendSmsIfNetwork(sender, smsSendBuffer);
    }
    else{
      // send SMS indicating failure
      snprintf(smsSendBuffer, SMS_BUFF_SIZE, "Failed to remove %s from %s", remove_num, list_name);
      sendSmsIfNetwork(sender, smsSendBuffer);
    }
  }
}

/*
 * removeNumListEntry()
 * Removes an entry from the specified list. Note, this does not protect against 
 * removing everyone from a list, which could be a problem for the AuthCommand list.
 */
boolean removeNumListEntry(char list[][25], uint8_t list_len, char *remove_num, uint16_t eeprom_start_addr){
  // FIXME: should write values to eeprom
  
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    // check if its the one we're supposed to remove
    if(strcmp(list[i],remove_num)){
      // remove them (set it to nul string) and return success
      list[i][0] = '\0';
      //EEPROM.update(eeprom_start_addr + PHONE_NUM_MAX_LEN*i, '\0');
      return true;
    }
  }
  // if we made it here we didn't find them in the list, return failure
  return false;
}

/*
 * countListEntries()
 * Counts the number of non-empty entries in the list
 */
uint8_t countListEntries(char list[][25], uint8_t list_len){
  uint8_t blankEntries = 0;
  
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    if(list[i][0] == '\0'){
      blankEntries++;
    }
  }
  return list_len - blankEntries;
}

/*
 * respondWithList()
 * sends an SMS with the contents of the specified list
 */
void respondWithList(char list[][25], uint8_t list_len, char *list_name){

  snprintf(smsSendBuffer, SMS_BUFF_SIZE, "%s: ", list_name);
  // loop through the list
  for( uint8_t i = 0; i < list_len; i++){
    // check if its the one we're supposed to remove
    if(list[i] != '\0'){
      if(i != 0){
        strncat(smsSendBuffer, ", ", 3);
      }
      strncat(smsSendBuffer, list[i], 11);
      return true;
    }
  }
  sendSmsIfNetwork(sender, smsSendBuffer);
}



