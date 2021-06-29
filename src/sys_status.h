#ifndef SYS_STATUS_H
#define SYS_STATUS_H

struct systemStatus_structure {  
  uint8_t structuresVersion;                        // Version of the data structures (system and data)
  uint8_t placeholder;                              // available for future use
  bool clockSet;                                    // Tells us if we need to connect and set the RTC
  bool connectedStatus;                             // Are we connected?
  bool verboseMode;                                 // Turns on extra messaging
  bool solarPowerMode;                              // Powered by a solar panel or utility power
  bool lowPowerMode;                                // Does the device need to run disconnected to save battery
  uint8_t lowBatteryMode;                           // Is the battery level so low that we can no longer connect
  int stateOfCharge;                                // Battery charge level
  uint8_t batteryState;                             // Stores the current battery state
  int resetCount;                                   // reset count of device (0-256)
  float timezone;                                   // Time zone value -12 to +12
  float dstOffset;                                  // How much does the DST value change?
  uint8_t openTime;                                 // Hour the park opens (0-23)
  uint8_t closeTime;                                // Hour the park closes (0-23)
  unsigned long lastHookResponse;                   // Last time we got a valid Webhook response
  unsigned long lastConnection;                     // Last time we successfully connected to Particle
  uint16_t lastConnectionDuration;                  // How long - in seconds - did it take to last connect to the Particle cloud
  float wateringThresholdPct;                       // When do we water
  int wateringDuration;                             // How long to water in seconds
};

extern struct systemStatus_structure sysStatus;

extern bool systemStatusWriteNeeded;               // Keep track of when we need to write

#endif
