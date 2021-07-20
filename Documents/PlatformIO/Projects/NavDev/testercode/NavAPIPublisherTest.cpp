/** =========================================================================
 * @file logging_to_ThingSpeak.ino
 * @brief Example logging data and publishing to ThingSpeak.
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @copyright (c) 2017-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for the Arduino IDE
//  NOTE:  These are ONLY needed to compile with the Arduino IDE.
//         If you use PlatformIO, you should set these build flags in your
//         platformio.ini
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif


/** End [defines] */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>


#include<Adafruit_GPS.h>
//import GPS library and assign serial port to GPS

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// To get all of the base classes for ModularSensors, include LoggerBase.
// NOTE:  Individual sensor definitions must be included separately.
#include <LoggerBase.h>
/** End [includes] */


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "Logging To Thingspeak with GPS as calculated variable";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "Navigator0";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -7;  // Mountain DLS time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const long   serialBaud = 115200;  // Baud rate for debugging
const int8_t greenLED   = 8;       // Pin for the green LED
const int8_t redLED     = 9;       // Pin for the red LED
const int8_t buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t wakePin    = -1;      // MCU interrupt/alarm pin to wake from sleep A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = -1;  // MCU pin controlling main sensor power
const int8_t threeSw = 22;
/** End [logging_options] */


// ==========================================================================
//  Wifi/Cellular Modem Options
// ==========================================================================
//#if not defined MS_BUILD_TESTING || defined MS_BUILD_TEST_XBEE_CELLULAR
/** Start [xbee_cell_transparent] */
// For any Digi Cellular XBee's
// NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global) can be used
// in either bypass or transparent mode, each with pros and cons
// The Telit based Digi XBees (LTE Cat1) can only use this mode.
#include <modems/DigiXBeeCellularTransparent.h>

// Create a reference to the serial port for the modem
// Extra hardware and software serial ports are created in the "Settings for
// Additional Serial Ports" section
HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
// AltSoftSerial &modemSerial = altSoftSerial;  // For software serial
// NeoSWSerial &modemSerial = neoSSerial1;  // For software serial
const long modemBaud = 9600;  // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
// The pin numbers here are for a Digi XBee with a Mayfly and LTE adapter
// For options https://github.com/EnviroDIY/LTEbee-Adapter/edit/master/README.md
const int8_t modemVccPin = A5;     // MCU pin controlling modem power
                                   // Option: modemVccPin = A5, if Mayfly SJ7 is
                                   // connected to the ASSOC pin
const int8_t modemStatusPin = 19;  // MCU pin used to read modem status
// NOTE:  If possible, use the `STATUS/SLEEP_not` (XBee pin 13) for status, but
// the CTS pin can also be used if necessary
const bool   useCTSforStatus = false;  // Flag to use the CTS pin for status
const int8_t modemResetPin   = 20;     // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;     // MCU pin for modem sleep/wake request
const int8_t modemLEDPin = redLED;     // MCU pin connected an LED to show modem
                                       // status

// Network connection information
const char* apn = "hologram";  // APN for GPRS connection

// Create the modem object
DigiXBeeCellularTransparent modemXBCT(&modemSerial, modemVccPin, modemStatusPin,
                                      useCTSforStatus, modemResetPin,
                                      modemSleepRqPin, apn);
// Create an extra reference to the modem by a generic name
DigiXBeeCellularTransparent modem = modemXBCT;
/** End [xbee_cell_transparent] */

// ==========================================================================
//  Setting up the GPS
// ==========================================================================
#include <AltSoftSerial.h>
AltSoftSerial GPSSerial;
//#define GPSSerial = altSoftSerial;
//AltSoftSerial &GPSSerial = altSoftSerial;
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

#define MS_NAVAPIPUBLISHER_DEBUG true

//uint32_t timer = millis();
// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */


// ==========================================================================
//  Atlas EZO RTD
// ==========================================================================
/** Start [atlas_rtd] */
/*
#include <sensors/AtlasScientificRTD.h>

const int8_t AtlasRTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
uint8_t      AtlasRTDi2c_addr = 0x66;         // Default for RTD is 0x66 (102)
// All Atlas sensors have different default I2C addresses, but any of them can
// be re-addressed to any 8 bit number.  If using the default address for any
// Atlas Scientific sensor, you may omit this argument.

// Create an Atlas Scientific RTD sensor object
#ifdef MS_ATLAS_SOFTWAREWIRE
AtlasScientificRTD atlasRTD(&softI2C, AtlasRTDPower, AtlasRTDi2c_addr);
// AtlasScientificRTD atlasRTD(AtlasRTDPower, softwareSDA, softwareSCL,
//                             AtlasRTDi2c_addr);
#else
// AtlasScientificRTD atlasRTD(AtlasRTDPower, AtlasRTDi2c_addr);
AtlasScientificRTD atlasRTD(AtlasRTDPower, AtlasRTDi2c_addr, 3);
#endif

// Create a temperature variable pointer for the RTD
Variable* atlasTemp = new AtlasScientificRTD_Temp(
    &atlasRTD, "12345678-abcd-1234-ef00-1234567890ab");
/** End [atlas_rtd] */

// ==========================================================================
//  Atlas EZO EC
// ==========================================================================
/** Start [atlas_ec] */
/*
#include <sensors/AtlasScientificEC.h>

const int8_t AtlasECPower    = sensorPowerPin;  // Power pin (-1 if unconnected)
uint8_t      AtlasECi2c_addr = 0x64;            // Default for EC is 0x64 (100)
// All Atlas sensors have different default I2C addresses, but any of them can
// be re-addressed to any 8 bit number.  If using the default address for any
// Atlas Scientific sensor, you may omit this argument.

// Create an Atlas Scientific Conductivity sensor object
#ifdef MS_ATLAS_SOFTWAREWIRE
// AtlasScientificEC atlasEC(AtlasECPower, softwareSDA, softwareSCL,
//                           AtlasECi2c_addr);
AtlasScientificEC atlasEC(&softI2C, AtlasECPower, AtlasECi2c_addr);
#else
// AtlasScientificEC atlasEC(AtlasECPower, AtlasECi2c_addr);
AtlasScientificEC atlasEC(AtlasECPower,
                          ATLAS_COND_I2C_ADDR,
                           3);
#endif

// Create four variable pointers for the EZO-ES
Variable* atlasCond = new AtlasScientificEC_Cond(
    &atlasEC, "12345678-abcd-1234-ef00-1234567890ab");
Variable* atlasTDS =
    new AtlasScientificEC_TDS(&atlasEC, "12345678-abcd-1234-ef00-1234567890ab");
Variable* atlasSal = new AtlasScientificEC_Salinity(
    &atlasEC, "12345678-abcd-1234-ef00-1234567890ab");
Variable* atlasGrav = new AtlasScientificEC_SpecificGravity(
    &atlasEC, "12345678-abcd-1234-ef00-1234567890ab");

// Create a calculated variable for the temperature compensated conductivity
// (that is, the specific conductance).  For this example, we will use the
// temperature measured by the Atlas RTD above this.  You could use the
// temperature returned by any other water temperature sensor if desired.
// **DO NOT** use your logger board temperature (ie, from the DS3231) to
// calculate specific conductance!
float calculateAtlasSpCond(void) {
    float spCond          = -9999;  // Always safest to start with a bad value
    float waterTemp       = atlasTemp->getValue();
    float rawCond         = atlasCond->getValue();
    float temperatureCoef = 0.019;
    // ^^ Linearized temperature correction coefficient per degrees Celsius.
    // The value of 0.019 comes from measurements reported here:
    // Hayashi M. Temperature-electrical conductivity relation of water for
    // environmental monitoring and geophysical data inversion. Environ Monit
    // Assess. 2004 Aug-Sep;96(1-3):119-28.
    // doi: 10.1023/b:emas.0000031719.83065.68. PMID: 15327152.
    if (waterTemp != -9999 &&
        rawCond != -9999)  // make sure both inputs are good
    {
        spCond = rawCond / (1 + temperatureCoef * (waterTemp - 25.0));
    }
    return spCond;
}

// Properties of the calculated variable
// The number of digits after the decimal place
const uint8_t atlasSpCondResolution = 0;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char* atlasSpCondName = "specificConductance";
// This must be a value from http://vocabulary.odm2.org/units/
const char* atlasSpCondUnit = "microsiemenPerCentimeter";
// A short code for the variable
const char* atlasSpCondCode = "atlasSpCond";
// The (optional) universallly unique identifier
const char* atlasSpCondUUID = "12345678-abcd-1234-ef00-1234567890ab";

// Finally, Create the specific conductance variable and return a pointer to it
Variable* atlasSpCond =
    new Variable(calculateAtlasSpCond, atlasSpCondResolution, atlasSpCondName,
                 atlasSpCondUnit, atlasSpCondCode, atlasSpCondUUID);
/** End [atlas_ec] */


// ==========================================================================
//  Atlas Scientific EZO-pH Sensor
// ==========================================================================
/** Start [atlas_ph] */
/*
#include <sensors/AtlasScientificpH.h>

const int8_t AtlaspHPower    = sensorPowerPin;  // Power pin (-1 if unconnected)
uint8_t      AtlaspHi2c_addr = 0x63;            // Default for pH is 0x63 (99)
// All Atlas sensors have different default I2C addresses, but any of them can
// be re-addressed to any 8 bit number.  If using the default address for any
// Atlas Scientific sensor, you may omit this argument.

// Create an Atlas Scientific pH sensor object
#ifdef MS_ATLAS_SOFTWAREWIRE
AtlasScientificpH atlaspH(&softI2C, AtlaspHPower, AtlaspHi2c_addr);
// AtlasScientificpH atlaspH(AtlaspHPower, softwareSDA, softwareSCL,
//                           AtlaspHi2c_addr);
#else
// AtlasScientificpH atlaspH(AtlaspHPower, AtlaspHi2c_addr);
AtlasScientificpH atlaspH(AtlaspHPower, AtlaspHi2c_addr, 3);
#endif

// Create a pH variable pointer for the pH sensor
Variable* atlaspHpH =
    new AtlasScientificpH_pH(&atlaspH, "12345678-abcd-1234-ef00-1234567890ab");
/** End [atlas_ph] */

// ==========================================================================
//  Creating the Variables associated with GPS
// ==========================================================================

float findLat(void){
    float lat = GPS.latitudeDegrees;
    return lat;
}
// Properties of the calculated variable
// The number of digits after the decimal place
const uint8_t gpsLatResolution = 5;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char* gpsLatName = "latitude";
// This must be a value from http://vocabulary.odm2.org/units/
const char* gpsLatUnit = "degree";
// A short code for the variable
const char* gpslatCode = "adagpsLat";
// The (optional) universallly unique identifier
const char* gpslatUUID = "lat";
// Finally, Create the specific conductance variable and return a pointer to it
Variable* gpsLat =
    new Variable(findLat, gpsLatResolution, gpsLatName,
                 gpsLatUnit, gpslatCode, gpslatUUID);

float findLon(void){
    float lon = (GPS.longitudeDegrees);
    return lon;
}
// The number of digits after the decimal place
const uint8_t gpsLonResolution = 5;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char* gpsLonName = "longitude";
// This must be a value from http://vocabulary.odm2.org/units/
const char* gpsLonUnit = "degree";
// A short code for the variable
const char* gpslonCode = "adagpsLon";
// The (optional) universallly unique identifier
const char* gpslonUUID = "lon";
// Finally, Create the specific conductance variable and return a pointer to it
Variable* gpsLon =
    new Variable(findLon, gpsLonResolution, gpsLonName,
                 gpsLonUnit, gpslonCode, gpslonUUID);

float findAlt(void){
    float alt = (GPS.altitude);
    return alt;
}
// The number of digits after the decimal place
const uint8_t gpsAltResolution = 5;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char* gpsAltName = "altitude";
// This must be a value from http://vocabulary.odm2.org/units/
const char* gpsAltUnit = "meters";
// A short code for the variable
const char* gpsAltCode = "adagpsAlt";
// The (optional) universallly unique identifier
const char* gpsAltUUID = "alt";
// Finally, Create the specific conductance variable and return a pointer to it
Variable* gpsAlt =
    new Variable(findAlt, gpsAltResolution, gpsAltName,
                 gpsAltUnit, gpsAltCode, gpsAltUUID);

float findSpd(void){
    float spdk = GPS.speed;
    float spd = spdk/1.944;
    return spd;
}
const uint8_t gpsSpdResolution = 5;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char* gpsSpdName = "speed";
// This must be a value from http://vocabulary.odm2.org/units/
const char* gpsSpdUnit = "metersPerSecond";
// A short code for the variable
const char* gpsSpdCode = "adaGpsSpd";
// The (optional) universallly unique identifier
const char* gpsSpdUUID = "spd";
// Finally, Create the specific conductance variable and return a pointer to it
Variable* gpsSpd =
    new Variable(findSpd, gpsSpdResolution, gpsSpdName,
                 gpsSpdUnit, gpsSpdCode, gpsSpdUUID);
// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
/*
Variable* variableList[] = {
    new AtlasScientificEC_Cond(
    &atlasEC, "12345678-abcd-1234-ef00-1234567890ab"),
    new AtlasScientificEC_TDS(&atlasEC, "12345678-abcd-1234-ef00-1234567890ab"),
    new AtlasScientificEC_Salinity(&atlasEC, "12345678-abcd-1234-ef00-1234567890ab"),
    new AtlasScientificEC_SpecificGravity(&atlasEC, "12345678-abcd-1234-ef00-1234567890ab"),
    new AtlasScientificRTD_Temp(
    &atlasRTD, "12345678-abcd-1234-ef00-1234567890ab"),
    new AtlasScientificpH_pH(&atlaspH, "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_Battery(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab")
    /*new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab")*///};
    // Count up the number of pointers in the array
//int variableCount = sizeof(variableList) / sizeof(variableList[0]);

Variable* gpsVariableList[] = {
    new Variable(findLat, gpsLatResolution, gpsLatName,
                 gpsLatUnit, gpslatCode, gpslatUUID),
    new Variable(findLon, gpsLonResolution, gpsLonName,
                 gpsLonUnit, gpslonCode, gpslonUUID),
    new Variable(findAlt, gpsAltResolution, gpsAltName,
                gpsAltUnit, gpsAltUUID),
    new Variable(findSpd, gpsSpdResolution, gpsSpdName,
                gpsSpdUnit, gpsSpdUUID)
};
// Count up the number of pointers in the array
int gpsVariableCount = sizeof(gpsVariableList) / sizeof(gpsVariableList[0]);
// Create the VariableArray object
//VariableArray varArray;
VariableArray gpsArray;
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a logger instance
//Logger dataLogger;
Logger gpsLogger;
/** End [loggers] */


// ==========================================================================
//  Creating Data Publisher[s]
// ==========================================================================
// Create a channel with fields on ThingSpeak in advance
// The fields will be sent in exactly the order they are in the variable array.
// Any custom name or identifier given to the field on ThingSpeak is irrelevant.
// No more than 8 fields of data can go to any one channel.  Any fields beyond
// the eighth in the array will be ignored.
/*
const char* thingSpeakMQTTKey =
    "UCVQCOD6UMMXO3NU";  // Your MQTT API Key from Account > MyProfile.
const char* thingSpeakChannelID =
    "1322279";  // The numeric channel id for your channel
const char* thingSpeakChannelKey =
    "H2X1OR5G80U3O9C9";  // The Write API Key for your channel
const char* thingSpeakChannelID1 =
    "1322280";  // The numeric channel id for your channel
const char* thingSpeakChannelKey1 =
    "Q21GGX5KDYCJPIKO";  // The Write API Key for your channel
*/
const char* deviceID = "devicetst";
// Create a data publisher for ThingSpeak
#include <publishers/NavAPIPublisher.h>
NavAPIPublisher tst;

/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate); 
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}

// Reads the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();
    
    // power up GPS
    pinMode(threeSw, OUTPUT);
    digitalWrite(threeSw, HIGH);

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);
    
    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    
    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

      // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ); // 100 millihertz update rate to conserve power
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 5 Hz echo rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // Attach the modem and information pins to the logger
    //dataLogger.attachModem(modem);
    gpsLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    //dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             //greenLED);
    gpsLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the variable array[s], logger[s], and publisher[s]
    //varArray.begin(variableCount, variableList);
    gpsArray.begin(gpsVariableCount, gpsVariableList);
    //dataLogger.begin(LoggerID, loggingInterval, &varArray);
    gpsLogger.begin(LoggerID, loggingInterval, &gpsArray);

    tst.begin(gpsLogger, &modem.gsmClient, deviceID);
    /*TsMqtt.begin(dataLogger, &modem.gsmClient, thingSpeakMQTTKey,
                 thingSpeakChannelID, thingSpeakChannelKey);
    TsMqtt1.begin(gpsLogger, &modem.gsmClient, thingSpeakMQTTKey,
                 thingSpeakChannelID1, thingSpeakChannelKey1);
*/
    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        //Serial.println(F("Setting up sensors..."));
        gpsArray.setupSensors();
    }

    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.0 || !gpsLogger.isRTCSane()) {
        // Synchronize the RTC with NIST
        // This will also set up the modem
        gpsLogger.syncRTC();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        //Serial.println(F("Setting up file on SD card"));
        gpsLogger.turnOnSDcard(
            true);  // true = wait for card to settle after power up
        gpsLogger.createLogFile(true);  // true = write a new header
        gpsLogger.turnOffSDcard(
            true);  // true = wait for internal housekeeping after write
    }

    // Call the processor sleep
    //Serial.println(F("Putting processor to sleep"));
    gpsLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
// Use this short loop for simple data logging and sending
void loop() {
    //if (GPSSerial.available()) {
        char c = GPS.read();
    //}
    if (GPSECHO)
        if (c) Serial.print(c);
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
    
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < 2.4) {
        gpsLogger.systemSleep();
    }
    else {        
        //dataLogger.publishDataToRemotes();
        gpsLogger.logDataAndPublish();
    }
    
}
/** End [loop] */