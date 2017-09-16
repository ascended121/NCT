This repo contains the code for UMD Balloon Payload Program's New Cell Tracker payload.

NCT is based off the Adafruit Ultimate GPS Shield (which provides logging and GPS) and the Adafruit FONA 2G module. The following code leverages the libraries that Adafruit provides for interacting with both of those modules.

The primary purpose of NCT is to determine its GPS location and to telemetry that to the ground via an SMS message, when network is available. As a secondary object, NCT also logs its position throughout the flight for flight post-processing and time-altitude correlation of data from other payloads on the flight.

This code implements two lists, one of phone numbers authorized to send commands to NCT, and one of phone numbers to which telemetry SMS should be sent at regular intervals. NCT has an SMS-based command interface which allows remote control of the payload during a flight. For additional details, refer to the BPP design revie documentation.
