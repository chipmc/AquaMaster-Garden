/*
* Project Aquamaster Garden Watering System
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:19 May 2021
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software will work with both pressure and PIR sensor counters
*/

/* Alert Count Definitions
* 0 = Normal Operations - No Alert
* 1 = Battery temp too high / low to charge
* 2 = Failed to connect to Particle 
* 3 = Failed to get Webhook response when connected
* 4 = Firmware update completed
* 5 = Firmware update timed out
* 6 = Firmware update failed
* 7 = Update attempt limit reached - done for the day
*/


//v1.00 - Based on Visitation Counter Project v11.04 as a start
//v1.01 - Updated with measurement bug fix
//v1.02 - Updated to improve ability to connect by increasing waitUntil() to 5 seconds
//v2.00 - Moved to a non-blocking connecting state
//v2.01 - Removed line that was restarting the session over and again.  
//v2.02 - Significant change - added Rachio Webhook
//v3.03 - Added stay awake
//v3.04 - Added the ability to set the duration - fixed logic and reporting
//v4.00 - Fixed a connectivity issue
//v5.00 - Added more reporting - stay awake longer at the top of the hour
//v5.01 - Added ability to set watering duration
//v6.00 - Moved to Visitation code base v31 as base.  Updated base code for battery management.  Also, set charging for smaller panel
//v6.02 - Working through minor bugs in code transition - Fixed state machine flow
//v7.00 - Fixing some minor notifications


// Particle Product definitions
PRODUCT_ID(PLATFORM_ID);                            // No longer need to specify - but device needs to be added to product ahead of time.
PRODUCT_VERSION(7);
#define DSTRULES isDSTusa
char currentPointRelease[6] ="7.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

struct currentStatus_structure {                    // currently 10 bytes long
  unsigned long lastCountTime;                      // When did we record our last count
  int soilMoisture;                                 // Soil moisture
  int solenoidState;                                // Water on or off
  int temperature;                                  // Current Temperature
  int alerts;                                       // What is the current alert count
  int maxMinValue;                                  // Highest count in one minute in the current period
  uint16_t maxConnectTime = 0;                      // Longest connect time for the day
  int minBatteryLevel = 100;                        // Lowest Battery level for the day
  uint8_t updateAttempts = 0;                       // Number of attempted updates each day
} current;


// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueuePosixRK.h"                    // Allows for queuing of messages - https://github.com/rickkas7/PublishQueuePosixRK

// Libraries with helper functions
#include "time_zone_fn.h"
#include "sys_status.h"

struct systemStatus_structure sysStatus;

// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 11 * 60;   // Timeout for trying to connect to Particle cloud in seconds
// If updating, we need to delay sleep in order to give the download time to come through before sleeping
const std::chrono::milliseconds firmwareUpdateMaxTime = 10min; // Set at least 5 minutes

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
FuelGauge fuelGauge;                                     // Enable the fuel gauge API      

// For monitoring / debugging, you can uncomment the next line
SerialLogHandler logHandler(LOG_LEVEL_ALL);

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, WATERING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, NAPPING_STATE, LOW_BATTERY_STATE, FIRMWARE_UPDATE};
char stateNames[11][17] = {"Initialize", "Error", "Idle", "Measuring", "Watering", "Connecting", "Reporting", "Response Wait", "Napping", "Low Battery", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};


// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int soilPin =       A0;                      // Pressure Sensor inerrupt pin

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 300000;         // In lowPowerMode, how long to stay awake every hour in millis
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
char wateringThresholdPctStr[8];
unsigned long connectionStartTime;

// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
bool firmwareUpdateInProgress = false;              // Helps us track if a firmware update is in progress
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char lowPowerModeStr[16];                           // In low power mode?
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
char currentOffsetStr[10];                          // What is our offset from UTC
char wateringDurationStr[16];                       // How long do we water
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;
bool volatile wateringTimerFlag = false;

// These variables are associated with the watchdog timer and will need to be better integrated
int outOfMemory = -1;
time_t RTCTime;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Interrupt Variables
volatile bool userSwitchDetect = false;              // Flag for a user switch press while in connected state

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
Timer awakeTimer(1800000, awakeTimerISR, true);           // 30 minute timer, calles the awakeTimerISR and is one-shot
Timer wateringTimer(600000, wateringTimerISR, true);      // 10 minute timer, calles the wateringTimerISR and is one-shot

void setup()                                        // Note: Disconnected Setup()
{
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  
  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  char responseTopic[125];
  String deviceID = System.deviceID();                                 // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);                             // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);       // Subscribe to the integration response event
  System.on(firmware_update, firmwareUpdateHandler);                   // Registers a handler that will track if we are getting an update
  System.on(out_of_memory, outOfMemoryHandler);                        // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

  Particle.variable("SoilMoisture", current.soilMoisture);             // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alerts);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("WateringPct",wateringThresholdPctStr);
  Particle.variable("WateringDuration",wateringDurationStr);

  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("SetWaterThreshold",setWaterThreshold);
  Particle.function("SetWaterDuration",setWaterDuration);


  // Particle and System Set up next
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  ab1805.withFOUT(D8).setup();                                         // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                         // Enable watchdog

  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
  }

  fuelGauge.wakeup();                                                  // Expliciely wake the Feul gauge and give it a half-sec
  delay(500);
  fuelGauge.quickStart();                                              // May help us re-establish a baseline for SoC

  // Next we will load FRAM and check or reset variables to their correct values
  fram.begin();                                                        // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) state = ERROR_STATE;         // Device will not work without FRAM
    else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::systemStatusAddr,sysStatus);                        // Loads the System Status array from FRAM
    fram.get(FRAM::currentCountsAddr,current);                         // Loead the current values array from FRAM
  }

  checkSystemValues();                                                 // Make sure System values are all in valid range

  if (current.updateAttempts >= 3) {
    char data[64];
    System.disableUpdates();                                           // We will only try to update three times in a day 
    current.alerts = 7;                                                // Set an alert that we have maxed out our updates for the day
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }

  // Next we set the timezone and check is we are in daylight savings time
  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  DSTRULES() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string
  
  // Publish Queue Posix is used exclusively for sending webhooks in order to conserve RAM and reduce writes / wear
  PublishQueuePosix::instance().setup();                               // Tend to the queue
  PublishQueuePosix::instance().withRamQueueSize(0);                   // Writes to memory immediately
  PublishQueuePosix::instance().withFileQueueSize(96);                 // This should last at least two days

  // Make up the strings to make console values easier to read
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over  
  if (Time.day() != Time.day(current.lastCountTime)) {                 // Check to see if the device was last on in a different day
    resetEverything();                                                 // Zero the counts for the new day
  }

  takeMeasurements();                                               // Populates values so you can read them before the hour

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day                                                            
    stayAwake = stayAwakeLong;                                        // Keeps Boron awake after reboot - helps with recovery
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  systemStatusWriteNeeded = true;                                      // Update FRAM with any changes from setup
  
  Log.info("Startup complete");
  digitalWrite(blueLED,LOW);                                           // Signal the end of startup
}


void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake && !current.solenoidState) state = NAPPING_STATE;         // When in low power mode, we can nap between taps
    if (firmwareUpdateInProgress) state= FIRMWARE_UPDATE;    
    if (Time.hour() != Time.hour(lastReportedTime)) state = MEASURING_STATE;                                                            // We want to report on the hour but not after bedtime
    break;

  case NAPPING_STATE: {                                                // This state puts the device in low power mode quickly
    if (state != oldState) publishStateTransition();
    if (wateringTimer.isActive() || awakeTimer.isActive())  break;    // Don't nap until we are done with watering
    if (sysStatus.connectedStatus || !Cellular.isOff()) disconnectFromParticle();           // Disconnect cleanly from Particle
    if (sysStatus.connectedStatus) disconnectFromParticle();           // Disconnect cleanly from Particle
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    ab1805.stopWDT();                                                  // If we are sleeping, we will miss petting the watchdog
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");
    else state = IDLE_STATE;
    } break;

    case MEASURING_STATE:

    if (state != oldState) publishStateTransition();
    takeMeasurements();

    if (Time.hour() > sysStatus.openTime && Time.hour() < sysStatus.closeTime && current.soilMoisture < sysStatus.wateringThresholdPct) {
      current.solenoidState = true;                                     // This sets a flag that indicates watering is needed
      currentCountsWriteNeeded = true;
    }
  
    state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (state != oldState) publishStateTransition();

    lastReportedTime = Time.now();                                    // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
    takeMeasurements();                                               // Take Measurements here for reporting
    if (Time.hour() == sysStatus.openTime) dailyCleanup();            // Once a day, clean house and publish to Google Sheets
    sendEvent();                                                      // Publish hourly but not at opening time as there is nothing to publish
    state = CONNECTING_STATE;                                         // We are only passing through this state once each hour    

    break;

  case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state
    static State retainedOldState;                                     // Keep track for where to go next (depends on whether we were called from Reporting)
    static unsigned long connectionStartTimeStamp;                     // Time in Millis that helps us know how long it took to connect

    if (state != oldState) {                                           // Non-blocking function - these are first time items
      retainedOldState = oldState;                                     // Keep track for where to go next
      sysStatus.lastConnectionDuration = 0;                            // Will exit with 0 if we do not connect or are connected or the connection time if we do
      publishStateTransition();

      // Let's make sure we need to connect
      if (sysStatus.connectedStatus && Particle.connected()) {
        Log.info("Connecting state but already connected");
        stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
        stayAwakeTimeStamp = millis();
        (retainedOldState = REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
        break;
      }
      // If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
      if (sysStatus.lowBatteryMode && digitalRead(userSwitch)) {
        Log.info("Connecting state but low battery mode");
        state = IDLE_STATE;
        break;
      }
      // If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
      if (sysStatus.lowPowerMode && digitalRead(userSwitch)) {         // Low power mode and user switch not pressed
        if (sysStatus.stateOfCharge <= 50 && (Time.hour() % 4)) {      // If the battery level is <50%, only connect every fourth hour
          Log.info("Connecting but <50%% charge - four hour schedule"); 
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here                                             
          break; 
        }                                                              // Leave this state and go connect - will return only if we are successful in connecting
        else if (sysStatus.stateOfCharge <= 65 && (Time.hour() % 2)) { // If the battery level is 50% -  65%, only connect every other hour
          Log.info("Connecting but 50-65%% charge - two hour schedule"); 
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here                                             
          break;                                                       // Leave this state and go connect - will return only if we are successful in connecting
        }
      }

      // OK, let's do this thing!
      connectionStartTimeStamp = millis();                             // Have to use millis as the clock will get reset on connect
      Cellular.on();                                                   // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
      Particle.connect();                                              // Told the Particle to connect, now we need to wait
    }

    sysStatus.lastConnectionDuration = int((millis() - connectionStartTimeStamp)/1000);

    if (Particle.connected()) {
      sysStatus.connectedStatus = true;
      sysStatus.lastConnection = Time.now();                           // This is the last time we attempted to connect
      stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      recordConnectionDetails();                                       // Record outcome of connection attempt
      Log.info("Cloud connection successful");
      attachInterrupt(userSwitch, userSwitchISR,FALLING);              // Attach interrupt for the user switch to enable verbose counts
      if (retainedOldState == REPORTING_STATE) state = RESP_WAIT_STATE;
      else state = IDLE_STATE;
    }
    else if (sysStatus.lastConnectionDuration > connectMaxTimeSec) {
      current.alerts = 2;                                              // Connection timed out alert
      sysStatus.connectedStatus = false;
      recordConnectionDetails();                                       // Record outcome of connection attempt
      Log.info("cloud connection unsuccessful");
      disconnectFromParticle();                                        // Make sure the modem is turned off
      if (sysStatus.solarPowerMode) setLowPowerMode("1");              // If we cannot connect, there is no point to stayng out of low power mode
      if ((Time.now() - sysStatus.lastConnection) > 3 * 3600L) {       // Only sends to ERROR_STATE if it has been over three hours - this ties to reporting and low battery state
        state = ERROR_STATE;     
        resetTimeStamp = millis();
        break;
      }
      else state = IDLE_STATE;
    } 
  } break;  

  case RESP_WAIT_STATE: {
    static unsigned long webhookTimeStamp = 0;                        // Webhook time stamp

    if (state != oldState) {
      webhookTimeStamp = millis();                                    // We are connected and we have published, head to the response wait state
      dataInFlight = true;                                            // set the data inflight flag
      publishStateTransition();
    }

    if (!dataInFlight)  {                                             // Response received --> on to the watering state
      state = WATERING_STATE;                                         // Remember we only get to this state if connected 
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset - this will pre-empt watering
      resetTimeStamp = millis();
      current.alerts = 3;                                             // Raise the missed webhook flag
      state = ERROR_STATE;                                            // Response timed out
    }
    currentCountsWriteNeeded = true;
    systemStatusWriteNeeded = true;

  } break;

  case WATERING_STATE: {                                                    // This state will examing soil values and decide on watering
    if (state != oldState) publishStateTransition();
    char data[32];

    if (current.solenoidState) {
      Log.info("Watering");
      wateringTimer.changePeriod(sysStatus.wateringDuration * 1000);
      snprintf(data, sizeof(data), "{\"duration\":%i}",sysStatus.wateringDuration);
      Particle.publish("Rachio-WaterGarden", data, PRIVATE);
    }
    else {
      Log.info("No watering needed at this time");
      if (sysStatus.wateringThresholdPct > 0) Particle.publish("Watering","No watering needed at this time",PRIVATE);
    }

    state = IDLE_STATE;
  }
  break;

  case ERROR_STATE:                                                    // To be enhanced - where we deal with errors
    if (state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {
      // The first condition implies that there is a connectivity issue - reset the modem
      if ((Time.now() - sysStatus.lastConnection) > 7200L) {           // It is been over two hours since we last connected to the cloud - time for a reset
        sysStatus.lastConnection = Time.now();                         // Make sure we don't do this very often
        disconnectFromParticle();                                      // Make sure cellular modem is off
        fram.put(FRAM::systemStatusAddr,sysStatus);
        Log.error("failed to connect to cloud, doing deep reset");
        delay(100);
        System.reset();                                                // Reset and reboot
      }
      // The next is also a simple reset but only until reset count = 3
      else if (sysStatus.resetCount <= 3) {                            // First try simple reset
        if (sysStatus.connectedStatus) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Error State - System Reset", PRIVATE, WITH_ACK);    // Brodcast Reset Action
        }
        delay(2000);
        System.reset();
      }
      // Once we have reset three times in one day, it is time for a full power cycle.
      else {                                                           // If we have had 3 resets - time to do something more
        if (sysStatus.connectedStatus) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Error State - Full Modem Reset", PRIVATE, WITH_ACK);            // Brodcast Reset Action
        }
        delay(2000);  
        disconnectFromParticle();                                      // Make sure we shut down connections gracefully
        sysStatus.resetCount = 0;                                      // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);                    // Won't get back to the main loop
        delay (100);
        ab1805.deepPowerDown();                                        // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
      }
    }
    break;

    case FIRMWARE_UPDATE: {
      static unsigned long stateTime;
      char data[64];

      if (state != oldState) {
        stateTime = millis();                                          // When did we start the firmware update?
        Log.info("In the firmware update state");
        publishStateTransition();
      }
      if (!firmwareUpdateInProgress) {                                 // Done with the update 
          Log.info("firmware update completed");
          state = IDLE_STATE;
      }
      else
      if (millis() - stateTime >= firmwareUpdateMaxTime.count()) {     // Ran out of time
          Log.info("firmware update timed out");
          current.alerts = 5;                                          // Record alert for timeout
          snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
          PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
          current.updateAttempts++;                                    // Increment the update attempt counter
          state = IDLE_STATE;
      }
    } break;
  }
  // Take care of housekeeping items here

  ab1805.loop();                                                       // Keeps the RTC synchronized with the Boron's clock

  PublishQueuePosix::instance().loop();                                // Check to see if we need to tend to the message queue


  if (systemStatusWriteNeeded) {                                       // These flags get set when a value is changed
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentCountsWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentCountsWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                              // In this function we are going to reset the system if there is an out of memory error
    char message[64];
    snprintf(message, sizeof(message), "Out of memory occurred size=%d",outOfMemory);
    Log.info(message);
    if (sysStatus.connectedStatus) {
      waitUntil(meterParticlePublish);
      Particle.publish("Memory",message,PRIVATE);                      // Publish to the console - this is important so we will not filter on verboseMod
    }
    delay(2000);
    System.reset();                                                    // An out of memory condition occurred - reset device.
  }

  if (sysStatus.connectedStatus && !Particle.connected()) {            // If the system thinks we are connected, let's make sure that we are
    state = CONNECTING_STATE;                                          // Go the connecting state - that way we will have limits on connection attempt duration
    sysStatus.connectedStatus = false;                                 // At least for now, this is the correct state value
    Log.info("Particle connection failed, reverting to the connecting state");
  }

  // End of housekeeping - end of main loop
}

void  recordConnectionDetails()  {                                     // Whether the connection was successful or not, we will collect and publish metrics
  char connectionStr[32];

  if (sysStatus.lastConnectionDuration > connectMaxTimeSec+1) sysStatus.lastConnectionDuration = 0;
  else if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day

  snprintf(connectionStr, sizeof(connectionStr),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
  Log.info(connectionStr);
  if (sysStatus.verboseMode && sysStatus.connectedStatus) {            // If we connected, let's publish the connection time
    waitUntil(meterParticlePublish);
    Particle.publish("Cellular",connectionStr,PRIVATE);
  }
  systemStatusWriteNeeded = true;
  currentCountsWriteNeeded = true;
}

void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue = Time.now();                                       // Going to start sending timestamps - and will modify for midnight to fix reporting issue
  snprintf(data, sizeof(data), "{\"soilMoisture\":%i, \"watering\":%i, \"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i, \"connecttime\":%i,\"timestamp\":%lu000}",current.soilMoisture, current.solenoidState, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alerts, sysStatus.lastConnectionDuration, timeStampValue);
  PublishQueuePosix::instance().publish("Ubidots-AquaMaster-Garden-v1", data, PRIVATE);
  current.alerts = 0;                                                 // Reset alerts after send
}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.verboseMode && sysStatus.connectedStatus) {
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", responseString, PRIVATE);
  }
}

/**
 * @brief The Firmware update handler tracks changes in the firmware update status
 * 
 * @details This handler is subscribed to in setup with System.on event and sets the firmwareUpdateinProgress flag that 
 * will trigger a state transition to the Firmware update state.  As some events are only see in this handler, failure
 * and success success codes are assigned here and the time out code in the main loop state.
 * 
 * @param event  - Firmware update 
 * @param param - Specific firmware update state
 */

void firmwareUpdateHandler(system_event_t event, int param) {
  switch(param) {
    char data[64];                                                     // Store the date in this character array - not global
      
    case firmware_update_begin:
      firmwareUpdateInProgress = true;
      break;
    case firmware_update_complete:
      firmwareUpdateInProgress = false;
      current.alerts = 4;                                              // Record a successful attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
      current.updateAttempts = 0;                                      // Zero the update attempts counter
      break;
    case firmware_update_failed:
      firmwareUpdateInProgress = false;
      current.alerts = 6;                                              // Record a failed attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publlish queue
      current.updateAttempts++;                                        // Increment the update attempts counter
      break;
  }
  currentCountsWriteNeeded = true;
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready

  getTemperature();                                                   // Get Temperature at startup as well
  
  // Battery Releated actions
  sysStatus.batteryState = System.batteryState();                      // Call before isItSafeToCharge() as it may overwrite the context

  isItSafeToCharge();                                                  // See if it is safe to charge

  if (sysStatus.lowPowerMode) {                                        // Need to take these steps if we are sleeping
    delay(500);
    fuelGauge.quickStart();                                            // May help us re-establish a baseline for SoC
    delay(500);
  }

  sysStatus.stateOfCharge = int(fuelGauge.getSoC());                   // Assign to system value

  if (sysStatus.stateOfCharge < 65 && sysStatus.batteryState == 1) {
    System.setPowerConfiguration(SystemPowerConfiguration());          // Reset the PMIC
  }

  if (sysStatus.stateOfCharge < current.minBatteryLevel) {
    current.minBatteryLevel = sysStatus.stateOfCharge;                 // Keep track of lowest value for the day
    currentCountsWriteNeeded = true;
  }
  
  if (sysStatus.stateOfCharge < 30) {
    sysStatus.lowBatteryMode = true;                                   // Check to see if we are in low battery territory
    if (!sysStatus.lowPowerMode) setLowPowerMode("1");                 // Should be there already but just in case...              
  }
  else sysStatus.lowBatteryMode = false;                               // We have sufficient to continue operations                          


  current.soilMoisture = map(analogRead(soilPin),0,3722,0,100);      // Sensor puts out 0-3V for 0% to 100% soil moisuture
  
  systemStatusWriteNeeded = true;
  currentCountsWriteNeeded = true;
}

bool isItSafeToCharge()                                               // Returns a true or false if the battery is in a safe charging range.  
{         
  PMIC pmic(true);                                 
  if (current.temperature < 36 || current.temperature > 100 )  {      // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    pmic.disableCharging();                                           // It is too cold or too hot to safely charge the battery
    sysStatus.batteryState = 1;                                       // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    return false;
  }
  else {
    pmic.enableCharging();                                            // It is safe to charge the battery
    return true;
  }
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {                                                // Get temperature and make sure we are not getting a spurrious value

  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  if (reading < 400) {                                                // This corresponds to 0 degrees - less than this and we should take another reading to be sure
    delay(50);
    reading = analogRead(tmp36Pin);
  }
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentCountsWriteNeeded=true;
  return current.temperature;
}


// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}

void wateringTimerISR() {
  current.solenoidState = false;
  state = MEASURING_STATE;
  currentCountsWriteNeeded = true;
}

void awakeTimerISR() {
  sysStatus.lowPowerMode = true;
  systemStatusWriteNeeded = true;
}

void userSwitchISR() {
  userSwitchDetect = true;                                            // The the flag for the user switch interrupt
}

// Power Management function
int setPowerConfig() {
  const int maxCurrentFromPanel = 340;                                // Set for implmentation (550mA for 3.5W Panel, 340 for 2W panel)
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration
  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(maxCurrentFromPanel) // Set maximum current the power source can provide  3.5W Panel (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(maxCurrentFromPanel) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
}


void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  if (sysStatus.connectedStatus) {
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Loading System Defaults", PRIVATE);
  }
  Log.info("Loading system defaults");
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.clockSet = false;
  sysStatus.lowBatteryMode = false;
  if (digitalRead(userSwitch)) setLowPowerMode("1");                  // Low power mode or not depending on user switch
  else setLowPowerMode("0");
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 0;
  sysStatus.closeTime = 24;
  sysStatus.solarPowerMode = true;  
  sysStatus.lastConnectionDuration = 0;                             // New measure
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

 /**
  * @brief This function checks to make sure all values that we pull from FRAM are in bounds
  * 
  * @details As new devices are comissioned or the sysStatus structure is changed, we need to make sure that values are 
  * in bounds so they do not cause unpredectable execution.
  * 
  */
void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range 
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
  if (sysStatus.lastConnectionDuration < 0 || sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = 0;
  sysStatus.solarPowerMode = true;                                  // Need to reset this value across the fleet
  if (sysStatus.wateringThresholdPct < 0 || sysStatus.wateringThresholdPct > 100) sysStatus.wateringThresholdPct = 10.0;
  if (sysStatus.wateringDuration < 0 || sysStatus.wateringDuration > 1000) sysStatus.wateringDuration = 300;
  if (current.maxConnectTime > connectMaxTimeSec) {
    current.maxConnectTime = 0;
    currentCountsWriteNeeded = true;
  }
  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

  // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct the strings that make the console easier to read
  * 
  * @details Looks at all the system setting values and creates the appropriate strings.  Note that this 
  * is a little inefficient but it cleans up a fair bit of code.
  * 
  */
void makeUpStringMessages() {
  // Special case for 24 hour operations
  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
    return;
  }
  // Open and Close Times
  snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);
  snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);

  // Low Power Mode String
  if (sysStatus.lowPowerMode) strncpy(lowPowerModeStr,"Low Power", sizeof(lowPowerModeStr));
  else strncpy(lowPowerModeStr,"Not Low Power", sizeof(lowPowerModeStr));

  // Watering Strings
  snprintf(wateringDurationStr,sizeof(wateringDurationStr),"%i seconds",sysStatus.wateringDuration);
  snprintf(wateringThresholdPctStr,sizeof(wateringThresholdPctStr),"%2.1f %%",sysStatus.wateringThresholdPct);

  return;
}


bool disconnectFromParticle()                                          // Ensures we disconnect cleanly from Particle
                                                                       // Updated based onthis thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  Log.info("In the disconnect from Particle function");
  Particle.disconnect();
  waitForNot(Particle.connected, 15000);                               // make sure before turning off the cellular modem
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  detachInterrupt(userSwitch);                                         // Stop watching the userSwitch as we will no longer be connected
  return true;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    ab1805.deepPowerDown(10);
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

/**
 * @brief Resets all counts to start a new day.
 * 
 * @details Once run, it will reset all daily-specific counts and trigger an update in FRAM.
 */
void resetEverything() {                                              // The device is waking up in a new day or is a new install
  char data[64];
  current.lastCountTime = Time.now();                                 // Set the time context to the new day
  sysStatus.resetCount = current.alerts = 0;                          // Reset everything for the day
  current.maxConnectTime = 0;                                         // Reset values for this time period
  current.minBatteryLevel = 100;
  currentCountsWriteNeeded = true;
  if (current.alerts ==7 || current.updateAttempts >=3) {             // We had tried to update enough times that we disabled updates for the day - resetting
    System.enableUpdates();
    current.alerts = 0;   
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }
  current.updateAttempts = 0;                                         // Reset the update attempts counter for the day
  currentCountsWriteNeeded=true;                                      // Make sure that the values are updated in FRAM
  systemStatusWriteNeeded=true;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (sysStatus.connectedStatus) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (sysStatus.connectedStatus) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}


/**
 * @brief Turns on/off verbose mode.
 * 
 * @details Extracts the integer command. Turns on verbose mode if the command is "1" and turns
 * off verbose mode if the command is "0".
 *
 * @param command A string with the integer command indicating to turn on or off verbose mode.
 * Only values of "0" or "1" are accepted. Values outside this range will cause the function
 * to return 0 to indicate an invalid entry.
 * 
 * @return 1 if successful, 0 if invalid command
 */
int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (sysStatus.connectedStatus) Particle.publish("Mode","Set Verbose Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (sysStatus.connectedStatus) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}

/**
 * @brief Returns a string describing the battery state.
 * 
 * @return String describing battery state.
 */
String batteryContextMessage() {
  return batteryContext[sysStatus.batteryState];
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  makeUpStringMessages();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  if (sysStatus.connectedStatus) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    Particle.publish("Time",data, PRIVATE, WITH_ACK);
  }
  return 1;
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  makeUpStringMessages();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (sysStatus.connectedStatus) Particle.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}


/**
 * @brief Toggles the device into low power mode based on the input command.
 * 
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    sysStatus.lowPowerMode = true;
    makeUpStringMessages();                                           // Updated system settings - refresh the string messages
    if (sysStatus.connectedStatus) {
      meterParticlePublish();
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    sysStatus.lowPowerMode = false;
    makeUpStringMessages();
    if (!sysStatus.connectedStatus) {                                 // In case we are not connected, we will do so now.
      state = CONNECTING_STATE;                                       // Will connect - if connection fails, will need to reset device
    }
    else Particle.publish("Mode",lowPowerModeStr, PRIVATE);
  }
  systemStatusWriteNeeded = true;
  return 1;
}

/**
 * @brief Let's you set the threashold for watering
 * 
 * @details Input the watering threshold in percent from 0 (bone dry) to 100 (soaking)
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setWaterThreshold(String command)                                  // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float tempThreshold = strtof(command,&pEND);                         // Looks for the first float and interprets it
  if ((tempThreshold < 0.0) | (tempThreshold > 100.0)) return 0;       // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringThresholdPct = tempThreshold;                      // debounce is how long we must space events to prevent overcounting
  systemStatusWriteNeeded = true;
  makeUpStringMessages();
  if (sysStatus.connectedStatus) {                                     // Publish result if feeling verbose
    if (sysStatus.wateringThresholdPct == 0) Particle.publish("System","Watering function disabled",PRIVATE);
    else Particle.publish("Threshold",wateringThresholdPctStr, PRIVATE);
  }
  return 1;                                                            // Returns 1 to let the user know if was reset
}

/**
 * @brief Let's you set the duration of the watering
 * 
 * @details Input the watering duration in seconds from 0 1000 seconds
 *
 * @param Pass the wating duration in seconds.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setWaterDuration(String command)                                   // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float tempValue = strtol(command,&pEND,10);                          // Looks for the first float and interprets it
  if ((tempValue < 0) | (tempValue > 1000)) return 0;                  // Make sure it falls in a valid range or send a "fail" result
  sysStatus.wateringDuration = tempValue;                              // debounce is how long we must space events to prevent overcounting
  systemStatusWriteNeeded = true;
  makeUpStringMessages();
  if (sysStatus.connectedStatus) {                                     // Publish result if feeling verbose
    Particle.publish("Duration",wateringDurationStr, PRIVATE);
  }
  return 1;                                                            // Returns 1 to let the user know if was reset
}



/**
 * @brief Publishes a state transition over serial and to the Particle monitoring system.
 * 
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if (sysStatus.verboseMode) {
    if (sysStatus.connectedStatus) Particle.publish("State Transition",stateTransitionString, PRIVATE, WITH_ACK);
    Log.info(stateTransitionString);
  }
}

/**
 * @brief Fully resets modem.
 * 
 * @details Disconnects from the cloud, resets modem and SIM, and deep sleeps for 10 seconds.
 * Adapted form Rikkas7's https://github.com/rickkas7/electronsample.
 */
void fullModemReset() {                                             // USed if we want to reset the modem's state
	Particle.disconnect(); 	                                          // Disconnect from the cloud    
	waitFor(Particle.connected, 15000);                               // Wait up to 15 seconds to disconnect
	// Reset the modem and SIM card
  Cellular.off();                                                   // Turn off the Cellular modem
  waitFor(Cellular.isOff, 30000);                                   // New feature with deviceOS@2.1.0

  ab1805.stopWDT();                                                 // No watchdogs interrupting our slumber
                                             
  config.mode(SystemSleepMode::ULTRA_LOW_POWER)
    .gpio(userSwitch,CHANGE)
    .duration(10 * 1000);


  System.sleep(config);                                             // Put the device to sleep device reboots from here   
  ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
}
/**
 * @brief Cleanup function that is run at the beginning of the day.
 * 
 * @details Syncs time with remote service and sets low power mode. Called from Reporting State ONLY.
 * Clean house at the end of the day
 */
void dailyCleanup() {
  Particle.publish("Daily Cleanup","Running", PRIVATE, WITH_ACK);            // Make sure this is being run
  sysStatus.verboseMode = false;
  Particle.syncTime();                                                 // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                // Wait for up to 30 seconds for the SyncTime to complete
  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }

  resetEverything();                                               // If so, we need to Zero the counts for the new day

  systemStatusWriteNeeded = true;
}