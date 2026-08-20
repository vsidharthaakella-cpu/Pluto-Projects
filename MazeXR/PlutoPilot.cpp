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
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based

}

// Distance (in cm) from an obstacle at which avoidance starts pushing the drone away.
static const uint16_t OBSTACLE_AVOID_DIST_CM = 100;

// ---- Maze-solving (reactive, greedy "move toward the most open side") ----
// Disabled for now -- this build is object-avoidance only. See plutoLoop()
// below for the commented-out maze logic.
/*
static const int16_t MAZE_MIN_CLEARANCE_MM = 150;
static const int16_t MAZE_MAX_VALID_RANGE_MM = 4000;
static const int16_t MAZE_OPEN_SENTINEL_MM   = 30000;
static const int16_t MAZE_STEP_RC = 30;
static const int16_t MAZE_ROLL_SIGN  = 1;    // + drives toward RIGHT
static const int16_t MAZE_PITCH_SIGN = 1;    // + drives toward FRONT

static int16_t mazeNormalizeRange ( int16_t rawMm ) {
  if ( rawMm < 0 || rawMm > MAZE_MAX_VALID_RANGE_MM ) return MAZE_OPEN_SENTINEL_MM;
  return rawMm;
}
*/

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Add your hardware initialization code here
  XRanging.init(LEFT, OBSTACLE_AVOID_DIST_CM);
  XRanging.init(RIGHT, OBSTACLE_AVOID_DIST_CM);
  XRanging.init(FRONT, OBSTACLE_AVOID_DIST_CM);
  XRanging.init(BACK, OBSTACLE_AVOID_DIST_CM);
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // do your one time stuffs here
  XRanging.initObjectAvoidance(OBSTACLE_AVOID_DIST_CM, LEFT, RIGHT, FRONT, BACK);
  XRanging.enableOA();
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  // Add your repeated code here

  /*
  int16_t leftMm  = mazeNormalizeRange ( XRanging.getRange ( LEFT ) );
  int16_t rightMm = mazeNormalizeRange ( XRanging.getRange ( RIGHT ) );
  int16_t frontMm = mazeNormalizeRange ( XRanging.getRange ( FRONT ) );
  int16_t backMm  = mazeNormalizeRange ( XRanging.getRange ( BACK ) );

  laser_e bestDirection = LEFT;
  int16_t bestMm = leftMm;
  if ( rightMm > bestMm ) { bestDirection = RIGHT; bestMm = rightMm; }
  if ( frontMm > bestMm ) { bestDirection = FRONT; bestMm = frontMm; }
  if ( backMm  > bestMm ) { bestDirection = BACK;  bestMm = backMm;  }

  if ( bestMm < MAZE_MIN_CLEARANCE_MM ) {
    // Boxed in on every side: hold position rather than push into a wall.
    RcCommand_Set ( RC_ROLL, 1500 );
    RcCommand_Set ( RC_PITCH, 1500 );
    return;
  }

  int16_t rollCmd  = 1500;
  int16_t pitchCmd = 1500;
  switch ( bestDirection ) {
    case LEFT:  rollCmd  = 1500 - MAZE_ROLL_SIGN  * MAZE_STEP_RC; break;
    case RIGHT: rollCmd  = 1500 + MAZE_ROLL_SIGN  * MAZE_STEP_RC; break;
    case FRONT: pitchCmd = 1500 + MAZE_PITCH_SIGN * MAZE_STEP_RC; break;
    case BACK:  pitchCmd = 1500 - MAZE_PITCH_SIGN * MAZE_STEP_RC; break;
    default: break;
  }

  RcCommand_Set ( RC_ROLL, rollCmd );
  RcCommand_Set ( RC_PITCH, pitchCmd );
  */
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  XRanging.disableOA();
}
