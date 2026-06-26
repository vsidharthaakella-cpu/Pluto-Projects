// Do not remove the include below
#include "PlutoPilot.h"

/**
 * Configures Pluto's receiver to use PPM or default ESP mode; activate the line matching your setup.
 * AUX channel configurations is only for PPM recievers if no custom configureMode function is called this are the default setup
 * ARM mode : Rx_AUX2, range 1300 to 2100
 * ANGLE mode : Rx_AUX2, range 900 to 2100
 * BARO mode : Rx_AUX3, range 1300 to 2100
 * MAG mode : Rx_AUX1, range 900 to 1300
 * HEADFREE mode : Rx_AUX1, range 1300 to 1700
 * DEV mode : Rx_AUX4, range 1500 to 2100
 */

// storing the previous altitude to compare with the current one
static int16_t previousAltitude = 0;
// current altitude value from the estimator
static int16_t currentAltitude = 0;
// vertical speed value from the estimator
static int16_t verticalVelocity = 0;
// difference between current and previous altitude
static int16_t altitudeJump = 0;
static int16_t triggerThreshold = 20;   // tune this
static int16_t takeoffHeight = 150;     
// altitude to hold after takeoff
static int16_t holdAltitude = 0;
// to make sure takeoff command runs only once
static bool takeoffTriggered = false;
// to make sure altitude hold gets locked only once
static bool holdLocked = false;

void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_PPM );    // PPM based
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Set the loop frequency for altitude monitoring.
  setUserLoopFrequency ( 100 );
}

void onLoopStartAutoInsertion ( void ) {
  // Turn on status led when developer mode starts.
  Set_LED ( STATUS, ON );
}

void onLoopStopAutoInsertion ( void ) {
  // Turn off status led when developer mode stops.
  Set_LED ( STATUS, OFF );
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // Reset runtime states before starting the loop.
  // taking the first altitude reading as the base value
  previousAltitude = Estimate_Get ( Position, Z );
  currentAltitude = previousAltitude;
  verticalVelocity = 0;
  altitudeJump = 0;
  holdAltitude = 0;
  takeoffTriggered = false;
  holdLocked = false;
  onLoopStartAutoInsertion();
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  // Read current altitude and vertical motion from the estimator.
  currentAltitude = Estimate_Get ( Position, Z );
  verticalVelocity = Estimate_Get ( Velocity, Z );

  // Detect sudden positive jump in altitude.
  altitudeJump = currentAltitude - previousAltitude;

  // Trigger takeoff only once after the altitude jump is detected.
  if ( altitudeJump > triggerThreshold && !takeoffTriggered ) {
    // arming only if the drone is ready and not armed already
    if ( FlightStatus_Check ( FS_OK_TO_ARM ) && !FlightStatus_Check ( FS_ARMED ) ) {
      Command_Arm ();
    }

    // sending the takeoff command with the preset height
    Command_TakeOff ( takeoffHeight );
    takeoffTriggered = true;
  }

  // Lock the current altitude once after takeoff is triggered.
  if ( takeoffTriggered && !holdLocked ) {
    // switching to altitude hold mode
    FlightMode_Set ( ATLTITUDEHOLD );
    // storing the altitude reached and telling the drone to hold there
    holdAltitude = Estimate_Get ( Position, Z );
    setDesiredPosition ( Z, holdAltitude );
    holdLocked = true;
  }

  // Update previous altitude for the next loop iteration.
  previousAltitude = currentAltitude;
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
  onLoopStartAutoInsertion();
  Command_Land(105);
}
