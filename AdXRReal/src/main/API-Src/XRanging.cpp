/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\XRanging.cpp                                       #
 #  Created Date: Sat, 31st Jan 2026                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Monday, 13th Aug 2026                                       #
 #  Modified By: Sidhartha Akella     
 #                                                                             #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  07/21:       What I targeted:                                              #
                  Object avoidance using more than one sensor,                 #
                  What problems I faced my logic made the drone think it had to always stabilize the drone. This meant it would react less to any object coming towards it because reacting meant losing stability and this would be overridden by the stabilization code. #
                  I implemented my own string theory where to stabilize the drone, I took the distance between any given object, and then used it to make a push center program. Where it calculates the center between four objects and centers. This meant that if we tried to use it to implement object avoidance, it would always try to find the center.#
                  There was a blind spot where the drone would not react if it was diagonal, and this meant it could hit a wall, and we would be cooked. #
                My solutions to the problems:
                  Firstly, I programmed the drone to turn on stabilization so to stabilize gyro, only when a safe distance away from all four objects
                  I made sure string theory was more tighter and would turn of to enable escape nudge 
                  My logic for this is if it determines that both sensors are reading the same input like an isosceles triangle use those two compute distances to drone and avoid.
                  Altitude to make sure it stays a certain height 
                What got carried over:
                  All my solutions got carried over

*******************************************************************************/

#include "API/API-Utils.h"
#include "API/Peripherals.h"
#include "API/XRanging.h"
#include "io/rc_controls.h"
#include "common/maths.h"
#include "config/runtime_config.h"

// flight/imu.h cannot be included stand-alone here: it assumes mw.cpp's full
// include order (sensors/acceleration.h, drivers/sensor.h, flight/pid.h,
// etc.) and pulls in the entire sensor/PID subsystem just to reach one
// struct.  Instead, declare the current-attitude global directly, matching
// flight/imu.h's rollAndPitchInclination_t byte-for-byte (see that header --
// keep this in sync if it ever changes).  Global data isn't name-mangled by
// extern "C" in the way functions are, but the wrapper is kept to mirror the
// original declaration exactly.
extern "C" {
  typedef struct rollAndPitchInclination_s {
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
  } rollAndPitchInclination_t_def;

  typedef union {
    int16_t raw [ 3 ];
    rollAndPitchInclination_t_def values;
  } rollAndPitchInclination_t;

  extern rollAndPitchInclination_t inclination;
}

// Bench-only instrumentation for props-off validation: RIGHT/centering
// direction sign and getRange() worst-case call latency. Flip to 0 and the
// entire block below compiles out -- no bench code reaches a flight build
// left at 0.
#define OA_BENCH_DEBUG 0// turn to zero
#if OA_BENCH_DEBUG
#include "API/Debugging.h"
static uint32_t oaMaxRangeCallUs [ 2 ] = { 0, 0 }; // [LEFT, RIGHT] worst-case getRange() duration, us.
#endif

// Laser sensor instances
static LaserSensor laserLEFT;
static LaserSensor laserRIGHT;
static LaserSensor laserFRONT;
static LaserSensor laserBACK;
static LaserSensor laserEXTERNAL;

/**
 * @brief Mapping structure between logical laser IDs and hardware.
 *
 * This structure maps:
 *  - Logical laser ID (LEFT, RIGHT, FRONT, BACK, EXTERNAL)
 *  - Status LED GPIO
 *  - XSHUT GPIO
 *  - Associated LaserSensor object
 *
 * This enables table-driven logic instead of switch-case handling.
 */
struct LaserMap {
  laser_e id;
  peripheral_gpio_pin_e ledGpio;
  peripheral_gpio_pin_e xshutGpio;
  LaserSensor *sensor;
};

// Laser mapping table
static LaserMap lasers [] = {
  { LEFT, GPIO_6, GPIO_7, &laserLEFT },
  { RIGHT, GPIO_10, GPIO_9, &laserRIGHT },
  { FRONT, GPIO_5, GPIO_16, &laserFRONT },
  { BACK, GPIO_8, GPIO_15, &laserBACK },
  { EXTERNAL, ( peripheral_gpio_pin_e ) -1, GPIO_14, &laserEXTERNAL },
};

// Calculate the number of elements in the 'lasers' array.
static constexpr uint8_t LASER_COUNT = sizeof ( lasers ) / sizeof ( lasers [ 0 ] );

// Function to check if a given GPIO pin is valid.
static inline bool isValidGpio ( peripheral_gpio_pin_e gpio ) {
  return ( gpio >= 0 && gpio < GPIO_COUNT );
}

/**
 * @brief Initializes the ranging sensors by resetting them, bringing them up one-by-one,
 * and performing an LED animation sequence.
 *
 * This function performs the following phases:
 * 1. **Reset Phase**: Turns off all lasers and LEDs by writing STATE_LOW to their respective GPIO pins.
 * 2. **Initialization Phase**: Powers on each laser module individually, initializes it, and assigns a unique I2C address.
 * 3. **LED Animation Phase**: Toggles LEDs in a blinking pattern for visual confirmation of initialization.
 */
void xRangingInit ( void ) {

  uint8_t i2cAddress = 42;

  // Phase 1: reset
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    Peripheral_Init ( lasers [ i ].xshutGpio, OUTPUT );
    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );

    if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
      Peripheral_Init ( lasers [ i ].ledGpio, OUTPUT );
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
    }
    delay ( 10 );
  }

  // Phase 2: bring up one-by-one
  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    laser_e id = lasers [ i ].id;
    if ( ! isXLaserInit [ id ] ) continue;

    Peripheral_Write ( lasers [ i ].xshutGpio, STATE_HIGH );
    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_HIGH );

    delay ( 30 );

    lasers [ i ].sensor->init ( );
    lasers [ i ].sensor->setAddress ( i2cAddress++ );

    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );

    delay ( 30 );
  }

  // Phase 3: LED animation
  for ( uint8_t blink = 0; blink < 4; blink++ ) {
    for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
      if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
      if ( isValidGpio ( lasers [ i ].ledGpio ) )
        Peripheral_Write ( lasers [ i ].ledGpio, STATE_TOGGLE );
    }
    delay ( 75 );
  }
}

// ============================================================
// XRanging public API
// ============================================================

void XRanging_P::init ( void ) {
  isXLaserInit [ LEFT ]      = true;
  triggerThreshold [ LEFT ]  = 200;
  isXLaserInit [ RIGHT ]     = true;
  triggerThreshold [ RIGHT ] = 200;
  isXLaserInit [ FRONT ]     = true;
  triggerThreshold [ FRONT ] = 200;
  isXLaserInit [ BACK ]      = true;
  triggerThreshold [ BACK ]  = 200;
}

void XRanging_P::init ( laser_e laser, int16_t threshold ) {
  isXLaserInit [ laser ]     = true;
  triggerThreshold [ laser ] = threshold * 10;
}

int16_t XRanging_P::getRange ( laser_e laser ) {
  if ( ! isXLaserInit [ laser ] ) return -1;

  for ( uint8_t i = 0; i < LASER_COUNT; i++ )
    if ( lasers [ i ].id == laser )
      return lasers [ i ].sensor->startRanging ( );

  return -1;
}

bool XRanging_P::isTriggered ( laser_e laser ) {
  if ( ! isXLaserInit [ laser ] ) return false;
  if ( triggerThreshold [ laser ] < 0 ) return false;

  int16_t range = getRange ( laser );
  if ( range < 0 ) return false;

  bool triggered = ( range < triggerThreshold [ laser ] );
  if ( isValidGpio ( lasers [ laser ].ledGpio ) )
    Peripheral_Write ( lasers [ laser ].ledGpio, triggered ? STATE_HIGH : STATE_LOW );

  return triggered;
}

XRanging_P XRanging;

// ============================================================
// Object Avoidance configuration
// ============================================================
// The ranging driver reports millimetres.  Keep this loop short: it runs in
// the flight-control path and must react to an approaching object, not a
// delayed moving-average value.
#define OA_SAMPLE_INTERVAL_MS       50U
#define OA_REACTION_TIME_MS         1200U

// Additional travel allowance for the aircraft's command, tilt and braking
// delay.  It is used only for a meaningful approach speed, so a stationary
// wall still uses the requested 100 cm stand-off.
#define OA_STOPPING_MARGIN_MM        500
#define OA_BRAKE_MARGIN_SPEED_MM_S   300
#define OA_MAX_PREDICTION_MM        2500
#define OA_MAX_CLOSING_SPEED_MM_S   3000
#define OA_MIN_VALID_RANGE_MM       30
#define OA_MAX_VALID_RANGE_MM       4000
#define OA_DISTANCE_JUMP_MARGIN_MM  100
#define OA_CENTER_DEADBAND_MM       50
#define OA_CENTER_EXIT_MARGIN_MM    100
// Treat two returns as opposing corridor boundaries only when their combined
// span is plausible.  This prevents a nearby side object plus a distant wall
// from being mistaken for a corridor that should be centered.
#define OA_CENTER_MAX_SPAN_MM       2600
#define OA_HARD_CLEARANCE_MM        300
// LEFT/RIGHT sensors are airframe-fixed, aimed horizontally along the lateral
// axis in level flight.  Roll rotates the airframe about the forward axis,
// which tilts that lateral beam toward the floor/ceiling, so the raw range
// to a roughly vertical wall is the slant distance (true horizontal distance
// / cos(roll)) rather than the horizontal clearance every OA threshold below
// assumes.  Uncorrected, a banked escape reads the wall it is turning toward
// as farther away than it really is, right when accuracy matters most.
// Pitch rotates about that same lateral axis the sensors point along, so it
// does not tilt them (to first order) and is intentionally not corrected for.
// Beyond this roll angle the beam may not be hitting the wall at all (aimed
// at the floor/ceiling); a cos() correction cannot recover a genuinely missed
// reading, so the sample is treated as invalid rather than over-corrected.
#define OA_TILT_MAX_VALID_DECIDEGREES 450   // 45.0 degrees
#define OA_SUDDEN_APPEAR_RC         25
#define OA_STATIONARY_RC            35
#define OA_CENTER_MAX_RC             55
#define OA_CENTER_DECEL_ZONE_MM     300
#define OA_SINGLE_BRAKE_START_MARGIN_MM 100
#define OA_SINGLE_BRAKE_END_MARGIN_MM   600
#define OA_SINGLE_BRAKE_GAIN_DIV        18
#define OA_SINGLE_BRAKE_MAX_RC          35
// Hysteresis for the single-side trigger boundary in escapeStrength(): once
// triggered, the object must retreat this far past dynamicBoundary before
// the side is considered clear. Without this, a static object sitting right
// at the standoff distance (the P-controller's own equilibrium point in
// singleSideHoldCommand()) flickers in/out of trigger on ordinary hover
// jitter, resetting triggerSamples and pulsing the push on/off. A moving
// object never sits on this edge (closing speed extends dynamicBoundary
// outward), so it is unaffected.
#define OA_SINGLE_TRIGGER_EXIT_MARGIN_MM 100
#define OA_SINGLE_HOLD_DEADBAND_MM      60
#define OA_SINGLE_HOLD_P_DIVISOR        10
#define OA_SINGLE_HOLD_D_DIVISOR        18
#define OA_SINGLE_HOLD_MAX_RC           55
#define OA_SINGLE_RETURN_MAX_RC         20
// The active angle controller saturates around 100 RC counts.  Keep normal
// OA within that real control range rather than issuing a saturated command.
// Reduced from 100: caps how hard/fast every avoidance maneuver -- escape,
// centering, hard-close -- is allowed to command, since 100 (full authority)
// was reacting more aggressively than wanted.
#define OA_MAX_RC                    70
// Normal side steering remains gentler than the hard-close escape above.
#define OA_NORMAL_MAX_RC              50
#define OA_RC_RAMP_PER_SAMPLE        25
// Hard-close may ramp faster, but still below the controller's saturation.
#define OA_RC_EMERGENCY_RAMP_PER_SAMPLE 75
// Sensor dropouts are routine on the VL53L0X (any non-zero RangeStatus yields
// -100 from the driver).  Coast on the last valid distance for this many
// consecutive invalid samples (4 * 50 ms = 200 ms) before declaring the side
// clear.  A single bad sample is noise, not "hazard cleared"; see C-102.
#define OA_MAX_COAST_SAMPLES          4
// Minimum closing-speed difference, mm/s, before speed is used to break a
// symmetric hard-close tie (see C-103).  closingSpeedMmS is an unfiltered
// single-interval estimate, so 1 mm of range noise is 20 mm/s of speed error;
// 400 mm/s keeps ordinary VL53L0X noise out of the tie-break.
#define OA_SPEED_TIEBREAK_MM_S      400
// Closing-speed to RC gain divisor: 1 RC count per 8 mm/s.  Same gain used by
// escapeStrength(), reused here so the two paths respond identically to speed.
#define OA_SPEED_GAIN_DIV             8
#define OA_USER_BLEND               0.25f

// Keep the established, unreversed body-frame mapping explicit.  LEFT must
// command a move to the right (+roll); RIGHT must command a move left (-roll).
#define OA_LEFT_ESCAPE_ROLL_SIGN      1
#define OA_RIGHT_ESCAPE_ROLL_SIGN    -1

// Modular two-sensor centering damping.  Set to 0 to restore the previous
// proportional-only centre command (errorMm / OA_CENTER_P_DIVISOR).
#define OA_ENABLE_CENTER_PD          1
#define OA_CENTER_P_DIVISOR          6
// Conservative D gain: its maximum contribution is 62 RC counts after rate
// limiting, so it damps a correction instead of becoming a second escape term.
#define OA_CENTER_D_DIVISOR          18
#define OA_CENTER_MAX_ERROR_RATE     2000

// Each side is enabled only after it has produced this many real, consecutive
// valid readings.  This prevents startup/I2C transients from commanding roll
// during takeoff without blocking a valid one-sided escape.
#define OA_SENSOR_READY_SAMPLES       6
#define OA_STATUS_BLINK_MS          200U
#define OA_TRIGGER_CONFIRM_SAMPLES    3
#define OA_FAST_TRIGGER_CONFIRM_SAMPLES 2
#define OA_GHOST_CONFIRM_SAMPLES      5
#define OA_HARD_CONFIRM_SAMPLES       2

static uint16_t _objectAoidDistance = 0; // Desired stand-off distance, mm.
static uint32_t lastAvoidanceRunMs  = 0;
static bool _enOA                   = false;

struct LeftAvoidanceState {
  int16_t lastDistanceMm;
  uint32_t lastSampleMs;
  bool hadValidSample;
  uint8_t validSamples;
  uint8_t invalidSamples;
  // Last accepted closing speed, mm/s, range 0..OA_MAX_CLOSING_SPEED_MM_S.
  // Held so a coasted (dropped) sample keeps the velocity-shifted trigger
  // boundary where it was instead of collapsing it to the static boundary.
  int32_t lastClosingSpeedMmS;
  int16_t rangeHistoryMm [ 3 ];
  uint8_t rangeHistoryCount;
  uint8_t rangeHistoryIndex;
  uint8_t triggerSamples;
  uint8_t hardCloseSamples;
  // Latched trigger state for the escapeStrength() hysteresis band -- see
  // OA_SINGLE_TRIGGER_EXIT_MARGIN_MM.
  bool triggerActive;
};

struct SideReading {
  int16_t distanceMm;
  int32_t closingSpeedMmS;
  int32_t separatingSpeedMmS;
  bool valid;
  bool newObject;
  bool fresh;
};

struct CenterControllerState {
  int32_t lastErrorMm;
  int32_t filteredErrorRateMmS;
  uint32_t lastUpdateMs;
  bool hasHistory;
};

enum OAMode {
  OA_MODE_CLEAR = 0,
  OA_MODE_SINGLE_LEFT,
  OA_MODE_SINGLE_RIGHT,
  OA_MODE_DUAL_CENTER
};

static LeftAvoidanceState leftOA  = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
static LeftAvoidanceState rightOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
static bool centerMode            = false;
static int16_t lastAvoidanceRc    = 0;
static bool leftSensorQualified   = false;
static bool rightSensorQualified  = false;
static CenterControllerState centerController = { 0, 0, 0, false };

// PITCH axis (BACK/FRONT) mirrors the ROLL axis (LEFT/RIGHT) state above,
// one-to-one -- see runAxisAvoidance().
static LeftAvoidanceState backOA  = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
static LeftAvoidanceState frontOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
static bool pitchCenterMode          = false;
static int16_t lastPitchAvoidanceRc  = 0;
static bool backSensorQualified      = false;
static bool frontSensorQualified     = false;
static CenterControllerState pitchCenterController = { 0, 0, 0, false };

#if OA_BENCH_DEBUG
// state: 0=clear, 1=commanding, 2=no qualified sensor, 3=Headfree inhibited.
static void reportOABench ( uint32_t now, const SideReading &left, const SideReading &right, int state ) {
  static uint32_t lastPrintMs = 0;
  if ( now - lastPrintMs < 500U ) return;
  lastPrintMs = now;

  Monitor_Print ( "OA L_mm", ( int ) left.distanceMm );
  Monitor_Print ( " Lv", ( int ) left.closingSpeedMmS );
  Monitor_Print ( " R_mm", ( int ) right.distanceMm );
  Monitor_Print ( " Rv", ( int ) right.closingSpeedMmS );
  Monitor_Print ( " Lq", ( int ) leftSensorQualified );
  Monitor_Print ( " Rq", ( int ) rightSensorQualified );
  Monitor_Print ( " state", state );
  Monitor_Print ( " hf", ( int ) ( FLIGHT_MODE ( HEADFREE_MODE ) != 0 ) );
  Monitor_Print ( " fm", ( int ) flightModeFlags );
  Monitor_Print ( " roll", ( int ) rcCommand [ ROLL ] );
  Monitor_Print ( " ctr", ( int ) centerMode );
  Monitor_Print ( " Lmax_us", ( int ) oaMaxRangeCallUs [ LEFT ] );
  Monitor_Println ( " Rmax_us", ( int ) oaMaxRangeCallUs [ RIGHT ] );
}
#endif

static void resetCenterController ( CenterControllerState &state ) {
  state = { 0, 0, 0, false };
}

static int16_t medianOfThree ( int16_t a, int16_t b, int16_t c ) {
  if ( a > b ) { int16_t t = a; a = b; b = t; }
  if ( b > c ) { int16_t t = b; b = c; c = t; }
  if ( a > b ) { int16_t t = a; a = b; b = t; }
  return b;
}

static void releaseAvoidanceAxis ( int16_t userInput, rc_alias axis, int16_t &lastRc ) {
  lastRc = 0;
  RC_ARRAY [ axis ] = 0;
  rcCommand [ axis ] = userInput;
  userRCflag [ axis ] = false;
}

static void showInterlockStatus ( uint32_t now, laser_e negLaser, laser_e posLaser ) {
  bool on = ( ( now / OA_STATUS_BLINK_MS ) & 1U ) != 0;
  Peripheral_Write ( lasers [ negLaser ].ledGpio, on ? STATE_HIGH : STATE_LOW );
  Peripheral_Write ( lasers [ posLaser ].ledGpio, on ? STATE_HIGH : STATE_LOW );
}

// Outer-loop PD centering.  P moves toward the wider side; D reduces roll as
// the left/right distance error is already closing, which limits overshoot.
static int16_t calculateCenterCommand ( int32_t errorMm, uint32_t now, CenterControllerState &state ) {
  int32_t absErrorMm = ABS ( errorMm );
  int32_t command = errorMm / OA_CENTER_P_DIVISOR;
  if ( absErrorMm < OA_CENTER_DECEL_ZONE_MM )
    command = command * absErrorMm / OA_CENTER_DECEL_ZONE_MM;

#if OA_ENABLE_CENTER_PD
  if ( state.hasHistory ) {
    uint32_t elapsedMs = now - state.lastUpdateMs;
    if ( elapsedMs > 0 ) {
      int32_t errorRate = ( errorMm - state.lastErrorMm ) * 1000 / ( int32_t ) elapsedMs;
      errorRate = constrain ( errorRate, -OA_CENTER_MAX_ERROR_RATE, OA_CENTER_MAX_ERROR_RATE );
      // A light 50/50 filter keeps D responsive enough to brake before center
      // while still rejecting one-sample ToF noise.
      state.filteredErrorRateMmS = ( state.filteredErrorRateMmS + errorRate ) / 2;
      command += state.filteredErrorRateMmS / OA_CENTER_D_DIVISOR;
    }
  }
#endif

  state.lastErrorMm = errorMm;
  state.lastUpdateMs = now;
  state.hasHistory = true;
  return constrain ( command, -OA_CENTER_MAX_RC, OA_CENTER_MAX_RC );
}

// Corrects a raw side-sensor slant range for airframe tilt; see
// OA_TILT_MAX_VALID_DECIDEGREES above.  LEFT/RIGHT point along the lateral
// axis and are foreshortened by ROLL (rotation about the forward axis tilts
// a lateral-pointing beam); BACK/FRONT point along the forward axis and are
// foreshortened by PITCH by the identical geometric argument (rotation about
// the lateral axis tilts a forward-pointing beam).  Yaw affects neither pair
// -- it's an azimuthal rotation, not a vertical tilt.  Returns false when the
// relevant angle is too steep to trust any correction, in which case the
// caller must treat the sample as invalid rather than act on an
// over-corrected value.
static bool tiltCorrectRangeMm ( int16_t rawMm, int16_t *correctedMm, laser_e laser ) {
  int16_t tiltDeciDegrees = ( laser == LEFT || laser == RIGHT )
                          ? inclination.values.rollDeciDegrees
                          : inclination.values.pitchDeciDegrees;
  if ( ABS ( tiltDeciDegrees ) >= OA_TILT_MAX_VALID_DECIDEGREES ) return false;

  float tiltRad = tiltDeciDegrees * ( 0.1f * RAD );
  int32_t corrected = ( int32_t ) ( rawMm * cos_approx ( tiltRad ) );
  *correctedMm = ( int16_t ) constrain ( corrected, 0, INT16_MAX );
  return true;
}

static SideReading readSide ( laser_e laser, LeftAvoidanceState *state, uint32_t now ) {
  SideReading reading = { 0, 0, 0, false, false, false };

#if OA_BENCH_DEBUG
  uint32_t callStartUs = micros ( );
#endif
  int16_t distance = isXLaserInit [ laser ] ? XRanging.getRange ( laser ) : -1;
#if OA_BENCH_DEBUG
  if ( isXLaserInit [ laser ] && ( laser == LEFT || laser == RIGHT ) ) {
    uint32_t callUs = micros ( ) - callStartUs;
    if ( callUs > oaMaxRangeCallUs [ laser ] ) oaMaxRangeCallUs [ laser ] = callUs;
  }
#endif

  // Correct for tilt-induced slant range before the raw sample is used for
  // anything downstream, so trigger confirmation, hard-close, closing-speed,
  // and centering all see the same tilt-corrected horizontal distance.  A
  // tilt angle too steep to trust (see OA_TILT_MAX_VALID_DECIDEGREES) is
  // pushed out of the valid range so it falls into the existing dropout/coast
  // path below, the same as any other unreliable sample.
  if ( distance >= OA_MIN_VALID_RANGE_MM && distance <= OA_MAX_VALID_RANGE_MM ) {
    int16_t corrected;
    distance = tiltCorrectRangeMm ( distance, &corrected, laser ) ? corrected : ( OA_MAX_VALID_RANGE_MM + 1 );
  }

  if ( distance < OA_MIN_VALID_RANGE_MM || distance > OA_MAX_VALID_RANGE_MM ) {
    if ( state->invalidSamples < 255 ) state->invalidSamples++;
    state->validSamples = 0;

    // C-102: a dropped sample is sensor noise, not "hazard cleared".  The
    // VL53L0X driver reports -100 for any non-zero RangeStatus (signal rate,
    // sigma, out of range, I2C error), which is routine against dark or
    // absorbent targets.  Coast on the last valid distance and the last
    // accepted closing speed for up to OA_MAX_COAST_SAMPLES (200 ms) so a
    // single dropout can neither release avoidance authority in one step nor
    // reset the ramp state.  Only after the coast window expires is the side
    // declared clear, and even then the release ramps through rampAvoidance().
    // lastSampleMs is deliberately NOT advanced while coasting: the next real
    // sample then measures its delta over the true elapsed interval, so the
    // recovered velocity estimate stays correct.
    if ( state->hadValidSample && state->invalidSamples <= OA_MAX_COAST_SAMPLES ) {
      reading.distanceMm = state->lastDistanceMm;
      reading.closingSpeedMmS = state->lastClosingSpeedMmS;
      reading.separatingSpeedMmS = 0;
      reading.valid = true;
      reading.newObject = false;
      reading.fresh = false;
      return reading;
    }

    state->hadValidSample = false;
    state->lastClosingSpeedMmS = 0;
    state->rangeHistoryCount = 0;
    state->rangeHistoryIndex = 0;
    state->triggerSamples = 0;
    state->hardCloseSamples = 0;
    return reading;
  }

  state->rangeHistoryMm [ state->rangeHistoryIndex++ ] = distance;
  if ( state->rangeHistoryIndex >= 3 ) state->rangeHistoryIndex = 0;
  if ( state->rangeHistoryCount < 3 ) state->rangeHistoryCount++;

  // A single bad ToF frame cannot move the median of three valid readings.
  reading.distanceMm = state->rangeHistoryCount == 3
                     ? medianOfThree ( state->rangeHistoryMm [ 0 ], state->rangeHistoryMm [ 1 ], state->rangeHistoryMm [ 2 ] )
                     : distance;
  reading.valid = true;
  reading.newObject = ! state->hadValidSample;
  reading.fresh = true;
  state->invalidSamples = 0;

  if ( state->hadValidSample ) {
    uint32_t elapsedMs = now - state->lastSampleMs;
    if ( elapsedMs > 0 ) {
      int32_t deltaMm = ( int32_t ) state->lastDistanceMm - reading.distanceMm;
      int32_t plausibleDeltaMm = OA_MAX_CLOSING_SPEED_MM_S * ( int32_t ) elapsedMs / 1000 + OA_DISTANCE_JUMP_MARGIN_MM;

      // Reject an isolated range jump as a velocity estimate.  The distance
      // itself is still used for close-range protection on this sample.
      if ( deltaMm <= plausibleDeltaMm && deltaMm >= -plausibleDeltaMm ) {
        // deltaMm > 0 means the object got closer.  A receding or noise-negative
        // sample must report 0, never a speed.  The subtraction below is done in
        // int32 on both operands: elapsedMs is cast, because an unsigned divisor
        // would promote a negative numerator to a huge positive value and the
        // constrain() lower bound would then never fire (05-review M1).  This is
        // load-bearing now that escapeStrength() also runs in centerMode (C-103).
        // Units: mm/s, range 0..OA_MAX_CLOSING_SPEED_MM_S.
        if ( deltaMm <= 0 ) {
          reading.closingSpeedMmS = 0;
          reading.separatingSpeedMmS = -deltaMm * 1000 / ( int32_t ) elapsedMs;
          reading.separatingSpeedMmS = constrain ( reading.separatingSpeedMmS, 0, OA_MAX_CLOSING_SPEED_MM_S );
        } else {
          reading.closingSpeedMmS = deltaMm * 1000 / ( int32_t ) elapsedMs;
          reading.closingSpeedMmS = constrain ( reading.closingSpeedMmS, 0, OA_MAX_CLOSING_SPEED_MM_S );
          reading.separatingSpeedMmS = 0;
        }
      } else {
        reading.newObject = true;
      }
    }
  }

  state->lastDistanceMm = reading.distanceMm;
  state->lastSampleMs = now;
  state->hadValidSample = true;
  state->lastClosingSpeedMmS = reading.closingSpeedMmS;
  if ( state->validSamples < 255 ) state->validSamples++;
  return reading;
}

static uint8_t requiredTriggerSamplesFor ( const SideReading &reading, const LeftAvoidanceState *state ) {
  uint8_t required = reading.closingSpeedMmS >= OA_BRAKE_MARGIN_SPEED_MM_S
                   ? OA_FAST_TRIGGER_CONFIRM_SAMPLES
                   : OA_TRIGGER_CONFIRM_SAMPLES;
  if ( state->validSamples < OA_GHOST_CONFIRM_SAMPLES && reading.closingSpeedMmS < OA_BRAKE_MARGIN_SPEED_MM_S )
    required = OA_GHOST_CONFIRM_SAMPLES;
  return required;
}

static int16_t escapeStrength ( const SideReading &reading, LeftAvoidanceState *state ) {
  if ( ! reading.valid ) {
    state->triggerSamples = 0;
    state->hardCloseSamples = 0;
    state->triggerActive = false;
    return 0;
  }

  // A high-power velocity reaction needs two consecutive plausible readings.
  // A close object is handled above without waiting for confirmation.
  int32_t predictedTravel = reading.closingSpeedMmS * OA_REACTION_TIME_MS / 1000;
  predictedTravel = constrain ( predictedTravel, 0, OA_MAX_PREDICTION_MM );
  int32_t dynamicBoundary = _objectAoidDistance + predictedTravel;
  if ( reading.closingSpeedMmS >= OA_BRAKE_MARGIN_SPEED_MM_S )
    dynamicBoundary += OA_STOPPING_MARGIN_MM;

  // Hysteresis: once latched active, the exit boundary is widened by
  // OA_SINGLE_TRIGGER_EXIT_MARGIN_MM so ordinary hover jitter right at
  // dynamicBoundary can't repeatedly flicker the trigger and reset
  // triggerSamples -- see the constant's comment above.
  int32_t exitBoundary = state->triggerActive ? dynamicBoundary + OA_SINGLE_TRIGGER_EXIT_MARGIN_MM : dynamicBoundary;
  bool inTrigger = reading.distanceMm <= exitBoundary;
  bool hardClose = reading.distanceMm <= OA_HARD_CLEARANCE_MM;

  if ( inTrigger && reading.fresh ) {
    if ( state->triggerSamples < 255 ) state->triggerSamples++;
  } else if ( ! inTrigger ) {
    state->triggerSamples = 0;
    state->triggerActive = false;
  }
  if ( hardClose && reading.fresh ) {
    if ( state->hardCloseSamples < 255 ) state->hardCloseSamples++;
  } else if ( ! hardClose ) {
    state->hardCloseSamples = 0;
  }

  // A one-frame near-zero return is a common optical/EMI fault.  It may only
  // produce a small nudge; normal avoidance needs 3 frames and hard-close
  // needs 2 frames before full authority is allowed.
  uint8_t requiredTriggerSamples = requiredTriggerSamplesFor ( reading, state );
  if ( hardClose && state->hardCloseSamples < OA_HARD_CONFIRM_SAMPLES ) return OA_SUDDEN_APPEAR_RC;
  if ( state->triggerSamples < requiredTriggerSamples ) return 0;
  state->triggerActive = true;
  if ( hardClose ) return OA_MAX_RC;

  int32_t distanceDeficit = _objectAoidDistance - reading.distanceMm;
  if ( distanceDeficit < 0 ) distanceDeficit = 0;
  int32_t strength = OA_STATIONARY_RC + reading.closingSpeedMmS / 8 + distanceDeficit / 2;
  return constrain ( strength, OA_STATIONARY_RC, OA_NORMAL_MAX_RC );
}

static int16_t singleSideBrakeStrength ( const SideReading &reading ) {
  if ( ! reading.valid || ! reading.fresh ) return 0;
  if ( reading.separatingSpeedMmS < OA_BRAKE_MARGIN_SPEED_MM_S ) return 0;

  int32_t brakeStartMm = _objectAoidDistance + OA_SINGLE_BRAKE_START_MARGIN_MM;
  int32_t brakeEndMm = _objectAoidDistance + OA_SINGLE_BRAKE_END_MARGIN_MM;
  if ( reading.distanceMm < brakeStartMm || reading.distanceMm > brakeEndMm ) return 0;

  int32_t brake = reading.separatingSpeedMmS / OA_SINGLE_BRAKE_GAIN_DIV;
  return constrain ( brake, 0, OA_SINGLE_BRAKE_MAX_RC );
}

static int16_t singleSideHoldCommand ( const SideReading &reading, int16_t sideSign, int16_t escapeStrength ) {
  if ( ! reading.valid ) return 0;

  bool hardClose = reading.distanceMm <= OA_HARD_CLEARANCE_MM;
  if ( hardClose )
    return sideSign * OA_MAX_RC;

  int32_t errorMm = ( int32_t ) _objectAoidDistance - reading.distanceMm;
  int32_t absErrorMm = ABS ( errorMm );
  int32_t commandMag = 0;

  if ( absErrorMm > OA_SINGLE_HOLD_DEADBAND_MM ) {
    commandMag = absErrorMm / OA_SINGLE_HOLD_P_DIVISOR;
    if ( errorMm > 0 ) {
      commandMag = constrain ( commandMag, 0, OA_SINGLE_HOLD_MAX_RC );
    } else {
      // Past the 100 cm target, only return gently.  This prevents the single-wall
      // controller from becoming a new shove toward the obstacle.
      commandMag = constrain ( commandMag, 0, OA_SINGLE_RETURN_MAX_RC );
      commandMag = -commandMag;
    }
  }

  // Closing speed adds escape authority; separating speed brakes the escape.  The
  // sign stays tied to the obstacle side, so LEFT returns positive roll and RIGHT
  // returns negative roll when too close.
  commandMag += reading.closingSpeedMmS / OA_SINGLE_HOLD_D_DIVISOR;
  commandMag -= reading.separatingSpeedMmS / OA_SINGLE_HOLD_D_DIVISOR;

  int32_t requested = sideSign * commandMag;
  if ( escapeStrength >= OA_MAX_RC ) requested = sideSign * OA_MAX_RC;
  int16_t maxRc = hardClose || escapeStrength >= OA_MAX_RC ? OA_MAX_RC : OA_SINGLE_HOLD_MAX_RC;
  return constrain ( requested, -maxRc, maxRc );
}

static int16_t blockUnsafeUserRoll ( int16_t userRoll, OAMode mode ) {
  switch ( mode ) {
    case OA_MODE_SINGLE_LEFT:
      return userRoll < 0 ? 0 : userRoll;
    case OA_MODE_SINGLE_RIGHT:
      return userRoll > 0 ? 0 : userRoll;
    case OA_MODE_DUAL_CENTER:
      return 0;
    default:
      return userRoll;
  }
}

// Scales hard-close escape authority by how much room the destination side
// actually has.  Picking a direction from relative position and then always
// commanding OA_MAX_RC ignored the destination's own absolute clearance: a
// wall at 350 mm is still nearly hard-close, and slamming to full authority
// toward it -- while escaping a 100 mm wall on the other side -- can run the
// aircraft into the second wall before its own hard-close/trigger debounce
// (OA_HARD_CONFIRM_SAMPLES / requiredTriggerSamplesFor) has a chance to react.
// Full authority is only granted once the destination clears back out to the
// configured stand-off distance; at or below OA_HARD_CLEARANCE_MM the
// destination is itself hard-close and has no usable headroom, so this
// returns 0 and the caller falls back to the closing-speed tie-break instead
// of trusting position alone in a corridor too tight to have a "far" side.
static int16_t hardEscapeMagnitude ( int16_t destinationDistanceMm ) {
  int32_t headroomMm = ( int32_t ) destinationDistanceMm - OA_HARD_CLEARANCE_MM;
  if ( headroomMm <= 0 ) return 0;

  int32_t fullHeadroomMm = ( int32_t ) _objectAoidDistance - OA_HARD_CLEARANCE_MM;
  if ( fullHeadroomMm <= 0 ) return OA_MAX_RC; // degenerate config: no scaling range to work with

  return ( int16_t ) constrain ( ( int32_t ) OA_MAX_RC * headroomMm / fullHeadroomMm, 0, OA_MAX_RC );
}

// Closing-speed tie-break for a hard-close pair where relative position gives
// no usable escape direction -- either a genuinely symmetric hard-close, or
// (via hardEscapeMagnitude above) a "farther" side with no headroom of its
// own.  Escapes from whichever side is actually closing, which is the only
// side lateral motion can help with; two stationary walls intentionally
// yield 0 here (see C-103).  speedDiffMmS > 0 means LEFT is closing faster ->
// positive (right) roll.
static int16_t hardCloseSpeedTiebreakRc ( const SideReading &left, const SideReading &right ) {
  int32_t speedDiffMmS = left.closingSpeedMmS - right.closingSpeedMmS;
  if ( speedDiffMmS > OA_SPEED_TIEBREAK_MM_S || speedDiffMmS < -OA_SPEED_TIEBREAK_MM_S )
    return ( int16_t ) constrain ( speedDiffMmS / OA_SPEED_GAIN_DIV, -OA_MAX_RC, OA_MAX_RC );
  return 0;
}

// Slew-limit every avoidance command, including hard-close commands, so an
// obstacle transition cannot reverse bank angle in one control sample.
static int16_t rampAvoidance ( int16_t requested, bool emergency, int16_t &lastRc ) {
  const int16_t step = emergency ? OA_RC_EMERGENCY_RAMP_PER_SAMPLE : OA_RC_RAMP_PER_SAMPLE;

  int16_t delta = requested - lastRc;
  delta = constrain ( delta, -step, step );
  lastRc += delta;
  return lastRc;
}

void XRanging_P::enableOA ( void ) {
  _enOA = true;
  leftOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  rightOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  backOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  frontOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  centerMode = false;
  pitchCenterMode = false;
  lastAvoidanceRc = 0;
  lastPitchAvoidanceRc = 0;
  leftSensorQualified = false;
  rightSensorQualified = false;
  backSensorQualified = false;
  frontSensorQualified = false;
  resetCenterController ( centerController );
  resetCenterController ( pitchCenterController );
}

void XRanging_P::disableOA ( void ) {
  _enOA = false;
  leftOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  rightOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  backOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  frontOA = { 0, 0, false, 0, 0, 0, { 0, 0, 0 }, 0, 0, 0, 0, false };
  centerMode = false;
  pitchCenterMode = false;
  lastAvoidanceRc = 0;
  lastPitchAvoidanceRc = 0;
  leftSensorQualified = false;
  rightSensorQualified = false;
  backSensorQualified = false;
  frontSensorQualified = false;
  resetCenterController ( centerController );
  resetCenterController ( pitchCenterController );

  // Release roll authority completely.  applyObjectAvoidance writes all three
  // of these on every commanding cycle (RC_ARRAY and rcCommand directly, plus
  // the flag), so all three must be released together.  Clearing only the flag
  // leaves the last avoidance roll latched in rcCommand [ ROLL ] until the next
  // annexCode pass, and leaves a non-zero RC_ARRAY [ ROLL ] that is re-applied
  // verbatim by mw.cpp the moment any other user-RC setter raises the flag
  // again.  Values are zeroed before the flag is lowered so that a consumer
  // racing this sequence reads a neutral command, never a stale one.
  // 0 is the neutral/centered value for both arrays.
  //
  // API CAVEAT (M-102, Gate 2 re-run 2026-08-07, accepted as documented
  // residual risk rather than fixed): disableOA() is a public API and can be
  // called from a sketch's plutoLoop() at any time, not only from this
  // driver's own onLoopFinish() path.  It unconditionally stomps
  // RC_ARRAY[ROLL], rcCommand[ROLL], and userRCflag[ROLL] regardless of who
  // last wrote them.  If a sketch is concurrently commanding ROLL directly
  // (e.g. RcCommand_Set(RC_ROLL, ...)) and then calls disableOA(), that
  // sketch's roll command is silently discarded -- and because
  // RcCommand_Set() is a latch, it stays discarded (not just for one cycle)
  // until the sketch writes ROLL again.  There is no ownership tracking here
  // to distinguish "avoidance's own roll authority" from "some other owner's
  // roll authority", and no error is returned to the caller.
  //   DO NOT command ROLL from a sketch while object avoidance is enabled;
  //   disableOA() will unconditionally clear the ROLL channel out from under
  //   any concurrent owner.  This is a known, accepted limitation, not an
  //   oversight -- see 04-changes.md FIX CYCLE 3 (M-102) for the ownership-
  //   tracking alternative that was considered and deferred.
  RC_ARRAY [ ROLL ] = 0;
  rcCommand [ ROLL ] = 0;
  userRCflag [ ROLL ] = false;
  // Same M-102 caveat applies to PITCH: released unconditionally, together
  // with ROLL, for the identical reasons documented above.
  RC_ARRAY [ PITCH ] = 0;
  rcCommand [ PITCH ] = 0;
  userRCflag [ PITCH ] = false;

  for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
    if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
    if ( isValidGpio ( lasers [ i ].ledGpio ) )
      Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
  }
}

void XRanging_P::initObjectAvoidance ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
  auto enableLaser = [ & ] ( laser_e l ) {
    if ( l < 0 ) return;
    isXLaserInit [ l ] = true;
  };

  _objectAoidDistance = _avoidDist * 10;
  enableLaser ( _laser1 );
  enableLaser ( _laser2 );
  enableLaser ( _laser3 );
  enableLaser ( _laser4 );
}

/**
 * @brief Applies velocity-aware avoidance from a negative/positive sensor
 * pair on one RC axis -- LEFT/RIGHT on ROLL, or BACK/FRONT on PITCH.
 *
 * With one object in range, the drone escapes away from that object.  When
 * both sensors are inside the requested stand-off, a proportional controller
 * centers the drone between them.  A hard-close reading overrides centering.
 * Mechanically identical control flow for both axes -- see applyObjectAvoidance().
 */
static void runAxisAvoidance ( laser_e negLaser, laser_e posLaser,
                                LeftAvoidanceState &negOA, LeftAvoidanceState &posOA,
                                bool &negQualified, bool &posQualified,
                                bool &centerModeRef, CenterControllerState &centerStateRef,
                                int16_t &lastRcRef, rc_alias rcAxis,
                                uint32_t now, int16_t userInput, bool reportBench ) {
  SideReading left = readSide ( negLaser, &negOA, now );
  SideReading right = readSide ( posLaser, &posOA, now );

  if ( ! negQualified && negOA.validSamples >= OA_SENSOR_READY_SAMPLES )
    negQualified = true;
  if ( ! posQualified && posOA.validSamples >= OA_SENSOR_READY_SAMPLES )
    posQualified = true;
  if ( negOA.invalidSamples > OA_MAX_COAST_SAMPLES ) negQualified = false;
  if ( posOA.invalidSamples > OA_MAX_COAST_SAMPLES ) posQualified = false;

  // The laser axes are fixed to the airframe.  Headfree rotates pilot input
  // by heading, so it is intentionally incompatible with this test-mode OA.
  // Release all overrides rather than mixing the two control frames.
  if ( FLIGHT_MODE ( HEADFREE_MODE ) ) {
    centerModeRef = false;
    resetCenterController ( centerStateRef );
    releaseAvoidanceAxis ( userInput, rcAxis, lastRcRef );
    // Alternating LEDs identify Headfree inhibition (not a detected obstacle).
    bool negOn = ( ( now / OA_STATUS_BLINK_MS ) & 1U ) != 0;
    Peripheral_Write ( lasers [ negLaser ].ledGpio, negOn ? STATE_HIGH : STATE_LOW );
    Peripheral_Write ( lasers [ posLaser ].ledGpio, negOn ? STATE_LOW : STATE_HIGH );
#if OA_BENCH_DEBUG
    if ( reportBench ) reportOABench ( now, left, right, 3 );
#endif
    return;
  }

  if ( ! negQualified && ! posQualified ) {
    centerModeRef = false;
    resetCenterController ( centerStateRef );
    releaseAvoidanceAxis ( userInput, rcAxis, lastRcRef );
    // Both LEDs blink together while neither side is ready.
    showInterlockStatus ( now, negLaser, posLaser );
#if OA_BENCH_DEBUG
    if ( reportBench ) reportOABench ( now, left, right, 2 );
#endif
    return;
  }

  // A side may independently escape from its own qualified sensor.  Only
  // centering requires two qualified, simultaneous measurements.
  if ( ! negQualified ) left.valid = false;
  if ( ! posQualified ) right.valid = false;

  int16_t requestedRc = 0;
  bool emergency = false;
  OAMode oaMode = OA_MODE_CLEAR;

  // Update per-side confidence before deciding whether two returns are a real
  // corridor.  Centering may use the far side as geometry, but the close side
  // must first be a confirmed trigger; otherwise two ghost returns can enter
  // "both wall" mode while the aircraft is actually in open space.
  int16_t leftEscapeStrength = escapeStrength ( left, &negOA );
  int16_t rightEscapeStrength = escapeStrength ( right, &posOA );
  int16_t escapeRc = constrain ( OA_LEFT_ESCAPE_ROLL_SIGN * ( int32_t ) leftEscapeStrength +
                                 OA_RIGHT_ESCAPE_ROLL_SIGN * ( int32_t ) rightEscapeStrength,
                                 -OA_MAX_RC, OA_MAX_RC );
  int16_t brakeRc = constrain ( -OA_LEFT_ESCAPE_ROLL_SIGN * ( int32_t ) singleSideBrakeStrength ( left ) +
                                -OA_RIGHT_ESCAPE_ROLL_SIGN * ( int32_t ) singleSideBrakeStrength ( right ),
                                -OA_SINGLE_BRAKE_MAX_RC, OA_SINGLE_BRAKE_MAX_RC );
  bool leftStandOffConfirmed = left.valid &&
                               left.distanceMm <= _objectAoidDistance &&
                               negOA.triggerSamples >= requiredTriggerSamplesFor ( left, &negOA );
  bool rightStandOffConfirmed = right.valid &&
                                right.distanceMm <= _objectAoidDistance &&
                                posOA.triggerSamples >= requiredTriggerSamplesFor ( right, &posOA );

  int32_t sensorSpanMm = ( int32_t ) left.distanceMm + right.distanceMm;
  bool pairedBoundaries = left.valid && right.valid && sensorSpanMm <= OA_CENTER_MAX_SPAN_MM;
  // Enter centering as soon as either side reaches the target.  For example,
  // 80 cm left and 120 cm right is a 200 cm corridor: move right until both
  // read about 100 cm, rather than first escaping LEFT as a single obstacle.
  bool enterCentering = pairedBoundaries &&
                         ( leftStandOffConfirmed || rightStandOffConfirmed );
  bool keepCentering = centerModeRef && pairedBoundaries &&
                       ( left.distanceMm <= _objectAoidDistance + OA_CENTER_EXIT_MARGIN_MM ||
                         right.distanceMm <= _objectAoidDistance + OA_CENTER_EXIT_MARGIN_MM );
  centerModeRef = enterCentering || keepCentering;

  // The velocity-based escape term.  Computed once, on the same escapeStrength()
  // path both modes now use.  C-103: at the live _objectAoidDistance = 1000 mm
  // centerMode is the DEFAULT indoor mode (any corridor or room narrower than
  // 2 m), and it previously never called escapeStrength() at all - i.e. the
  // entire velocity feature was bypassed in normal operation.  Positive command
  // moves away from the negative sensor; negative command moves away from the
  // positive sensor.
  bool hardClose = ( left.valid && left.distanceMm <= OA_HARD_CLEARANCE_MM ) ||
                   ( right.valid && right.distanceMm <= OA_HARD_CLEARANCE_MM );

  if ( ! centerModeRef || hardClose ) resetCenterController ( centerStateRef );

  if ( centerModeRef ) {
    oaMode = OA_MODE_DUAL_CENTER;
    if ( hardClose ) {
      // Centering cannot take priority over an imminent side collision.
      int32_t hardErrorMm = ( int32_t ) right.distanceMm - left.distanceMm;
      emergency = true;
      if ( hardErrorMm > OA_CENTER_DEADBAND_MM ) {
        // The positive-sensor side is farther, but "farther" is not "safe":
        // scale by its own headroom rather than assuming it can take
        // OA_MAX_RC (see hardEscapeMagnitude).
        requestedRc = hardEscapeMagnitude ( right.distanceMm );
      } else if ( hardErrorMm < -OA_CENTER_DEADBAND_MM ) {
        requestedRc = ( int16_t ) -hardEscapeMagnitude ( left.distanceMm );
      }
      // Symmetric hard-close (|hardErrorMm| <= deadband), or a "farther" side
      // that turned out to have no headroom of its own: position gives no
      // usable direction either way, so fall back to the closing-speed
      // tie-break.  Previously a true symmetric hard-close commanded nothing
      // at all and fell through to the release path - avoidance went silent,
      // and the LEDs went dark, with an object at 300 mm and closing (C-103).
      if ( requestedRc == 0 ) {
        requestedRc = hardCloseSpeedTiebreakRc ( left, right );
      }
    } else {
      int32_t errorMm = ( int32_t ) right.distanceMm - left.distanceMm;
      if ( errorMm > -OA_CENTER_DEADBAND_MM && errorMm < OA_CENTER_DEADBAND_MM ) errorMm = 0;
      int16_t centerRc = calculateCenterCommand ( errorMm, now, centerStateRef );

      // Static paired boundaries use the center command exclusively.  Escape
      // priority returns only for a meaningful closing speed; otherwise an
      // 80/120 cm corridor would repeatedly be treated as a negative-side-only
      // hazard and pendulum past its 100/100 cm midpoint.
      bool rapidApproach = left.closingSpeedMmS >= OA_BRAKE_MARGIN_SPEED_MM_S ||
                           right.closingSpeedMmS >= OA_BRAKE_MARGIN_SPEED_MM_S;
      requestedRc = ( rapidApproach && ABS ( escapeRc ) > ABS ( centerRc ) ) ? escapeRc : centerRc;
    }
  } else {
    if ( leftEscapeStrength != 0 && rightEscapeStrength != 0 ) {
      requestedRc = escapeRc != 0 ? escapeRc : brakeRc;
      oaMode = escapeRc > 0 ? OA_MODE_SINGLE_LEFT : OA_MODE_SINGLE_RIGHT;
    } else if ( leftEscapeStrength != 0 || ( left.valid && left.distanceMm <= _objectAoidDistance + OA_SINGLE_BRAKE_END_MARGIN_MM ) ) {
      requestedRc = singleSideHoldCommand ( left, OA_LEFT_ESCAPE_ROLL_SIGN, leftEscapeStrength );
      oaMode = requestedRc != 0 || left.distanceMm <= _objectAoidDistance + OA_SINGLE_BRAKE_END_MARGIN_MM
             ? OA_MODE_SINGLE_LEFT
             : OA_MODE_CLEAR;
    } else if ( rightEscapeStrength != 0 || ( right.valid && right.distanceMm <= _objectAoidDistance + OA_SINGLE_BRAKE_END_MARGIN_MM ) ) {
      requestedRc = singleSideHoldCommand ( right, OA_RIGHT_ESCAPE_ROLL_SIGN, rightEscapeStrength );
      oaMode = requestedRc != 0 || right.distanceMm <= _objectAoidDistance + OA_SINGLE_BRAKE_END_MARGIN_MM
             ? OA_MODE_SINGLE_RIGHT
             : OA_MODE_CLEAR;
    } else {
      requestedRc = 0;
      oaMode = OA_MODE_CLEAR;
    }
    emergency = hardClose;
  }

  // The limiter is always in the path, including on release.  C-102: this used
  // to be an early return that assigned lastRcRef = 0 directly, so a
  // single dropped sensor sample collapsed the output from up to 350 to 0 in
  // one 50 ms sample AND destroyed the ramp state, forcing recovery to climb
  // back from zero even though the hazard was still present.  With recurring
  // dropouts that capped the achievable authority at OA_RC_RAMP_PER_SAMPLE.
  // Now a zero request ramps down through the same limiter, and the pilot is
  // only released once the avoidance term has actually reached 0.
  int16_t avoidRc = rampAvoidance ( requestedRc, emergency, lastRcRef );

  if ( requestedRc == 0 && avoidRc == 0 ) {
    // No triggered hazard and nothing left to ramp down: do not override the
    // pilot or inject an RC command.  This is especially important during
    // take-off, when both sensors should be passive until a real trigger
    // condition is reached.
    //
    // RC_ARRAY [ rcAxis ] is cleared and the flag lowered (values before flag,
    // so a consumer racing this sequence reads a neutral value and never a
    // stale one) because this function writes RC_ARRAY directly on every
    // commanding cycle and mw.cpp re-applies a non-zero RC_ARRAY [ rcAxis ]
    // verbatim the moment any user-RC setter raises the flag again.
    //
    // rcCommand [ rcAxis ] is restored to the PILOT's command for this cycle,
    // not zeroed (M-101 / M-103).  applyObjectAvoidance() is the tail
    // statement of annexCode() (mw.cpp:492) and annexCode() recomputes
    // rcCommand [ rcAxis ] from rcData ~140 lines earlier in the SAME call, so
    // rcCommand [ rcAxis ] here is always this cycle's fresh pilot command -
    // it can never hold a stale avoidance value, and writing 0 discarded live
    // pilot input on nearly every 50 ms tick of normal flight.  userInput is
    // that captured value.
    releaseAvoidanceAxis ( userInput, rcAxis, lastRcRef );
    Peripheral_Write ( lasers [ negLaser ].ledGpio, STATE_LOW );
    Peripheral_Write ( lasers [ posLaser ].ledGpio, STATE_LOW );
#if OA_BENCH_DEBUG
    if ( reportBench ) reportOABench ( now, left, right, 0 );
#endif
    return;
  }

  if ( centerModeRef && ! hardClose ) {
    // Both LEDs on is the observable two-sensor centering state.
    Peripheral_Write ( lasers [ negLaser ].ledGpio, STATE_HIGH );
    Peripheral_Write ( lasers [ posLaser ].ledGpio, STATE_HIGH );
  } else {
    Peripheral_Write ( lasers [ negLaser ].ledGpio, left.valid && avoidRc > 0 ? STATE_HIGH : STATE_LOW );
    Peripheral_Write ( lasers [ posLaser ].ledGpio, right.valid && avoidRc < 0 ? STATE_HIGH : STATE_LOW );
  }

  // Block the pilot's own stick from countering avoidance in the dangerous
  // direction (e.g. pushing further into a wall this cycle is escaping from).
  // A pilot pushing away from the hazard passes through unblocked.
  int16_t safeUserInput = blockUnsafeUserRoll ( userInput, oaMode );

  // Write rcCommand now because annexCode has already run in this control
  // cycle; RC_ARRAY also keeps the user-code command path coherent next cycle.
  int32_t commandedAxis = ( int32_t ) ( safeUserInput * OA_USER_BLEND ) + avoidRc;
  RC_ARRAY [ rcAxis ] = constrain ( commandedAxis, -500, 500 );
  rcCommand [ rcAxis ] = RC_ARRAY [ rcAxis ];
  userRCflag [ rcAxis ] = true;

#if OA_BENCH_DEBUG
  if ( reportBench ) reportOABench ( now, left, right, 1 );
#endif
}

void applyObjectAvoidance ( void ) {
  uint32_t now = millis ( );
  if ( now - lastAvoidanceRunMs < OA_SAMPLE_INTERVAL_MS ) return;
  lastAvoidanceRunMs = now;
  if ( ! _enOA ) return;

  // annexCode has already converted RC input to the configured rate/expo
  // command in this control cycle.  Preserve that command whenever clear.
  int16_t userRoll = rcCommand [ ROLL ];
  int16_t userPitch = rcCommand [ PITCH ];

  runAxisAvoidance ( LEFT, RIGHT, leftOA, rightOA, leftSensorQualified, rightSensorQualified,
                     centerMode, centerController, lastAvoidanceRc, ROLL, now, userRoll, true );

  // Gate the PITCH axis on BACK/FRONT actually having been requested via
  // initObjectAvoidance().  Without this, an untouched runAxisAvoidance would
  // still blink the BACK/FRONT status LEDs and write RC_ARRAY[PITCH] every
  // cycle even when the caller only asked for LEFT/RIGHT (today's only real
  // usage) -- and BACK/FRONT's LED GPIOs currently alias the external flash
  // chip's SPI2 bus on every valid board target (see this session's hardware
  // review), so touching those pins unconditionally is a real wiring hazard,
  // not just an unused code path.
  if ( isXLaserInit [ BACK ] || isXLaserInit [ FRONT ] ) {
    runAxisAvoidance ( BACK, FRONT, backOA, frontOA, backSensorQualified, frontSensorQualified,
                       pitchCenterMode, pitchCenterController, lastPitchAvoidanceRc, PITCH, now, userPitch, false );
  }
}

// #include "API/API-Utils.h"
// #include "API/Peripherals.h"
// #include "API/XRanging.h"
// #include "io/rc_controls.h"
// #include "common/maths.h"

//           /* ============================================================
//            * Laser sensor instances
//            * ============================================================ */
//           static LaserSensor laserLEFT;
// static LaserSensor laserRIGHT;
// static LaserSensor laserFRONT;
// static LaserSensor laserBACK;
// static LaserSensor laserEXTERNAL;

// /* ============================================================
//  * Laser mapping table
//  * - Defines GPIOs and sensor object per logical laser
//  * - This enables table-driven logic instead of switch-case hell
//  * ============================================================ */
// struct LaserMap {
//   laser_e id;                         // Logical laser ID
//   peripheral_gpio_pin_e ledGpio;      // Status LED GPIO (-1 if unused)
//   peripheral_gpio_pin_e xshutGpio;    // XSHUT GPIO
//   LaserSensor *sensor;                // Sensor object
// };

// static LaserMap lasers [] = {
//   { LEFT, GPIO_6, GPIO_7, &laserLEFT },
//   { RIGHT, GPIO_10, GPIO_9, &laserRIGHT },
//   { FRONT, GPIO_5, GPIO_16, &laserFRONT },
//   { BACK, GPIO_8, GPIO_15, &laserBACK },
//   { EXTERNAL, ( peripheral_gpio_pin_e ) -1, GPIO_14, &laserEXTERNAL },
// };

// static constexpr uint8_t LASER_COUNT = sizeof ( lasers ) / sizeof ( lasers [ 0 ] );

// /* ============================================================
//  * Helper: validate GPIO before touching hardware
//  * ============================================================ */
// static inline bool isValidGpio ( peripheral_gpio_pin_e gpio ) {
//   return ( gpio >= 0 && gpio < GPIO_COUNT );
// }

// /* ============================================================
//  * XRanging low-level init
//  * - Powers up sensors one-by-one
//  * - Assigns unique I2C addresses
//  * ============================================================ */
// void xRangingInit ( void ) {

//   uint8_t i2cAddress = 42;

//   // Phase 1: Reset all enabled sensors (XSHUT LOW)
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     laser_e id = lasers [ i ].id;
//     if ( ! isXLaserInit [ id ] ) continue;

//     Peripheral_Init ( lasers [ i ].xshutGpio, OUTPUT );
//     Peripheral_Write ( lasers [ i ].xshutGpio, STATE_LOW );

//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Init ( lasers [ i ].ledGpio, OUTPUT );
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }

//     delay ( 10 );
//   }

//   // Phase 2: Bring up sensors one at a time
//   //           → init
//   //           → assign unique I2C address
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     laser_e id = lasers [ i ].id;
//     if ( ! isXLaserInit [ id ] ) continue;

//     Peripheral_Write ( lasers [ i ].xshutGpio, STATE_HIGH );
//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_HIGH );
//     }

//     delay ( 30 );

//     lasers [ i ].sensor->init ( );
//     lasers [ i ].sensor->setAddress ( i2cAddress++ );

//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }

//     delay ( 30 );
//   }

//   // Phase 3: Startup LED animation (LEFT/RIGHT/FRONT/BACK)
//   for ( uint8_t blink = 0; blink < 4; blink++ ) {
//     for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//       if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
//       if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//         Peripheral_Write ( lasers [ i ].ledGpio, STATE_TOGGLE );
//       }
//     }
//     delay ( 75 );
//   }
// }

// /* ============================================================
//  * XRanging public API
//  * ============================================================ */
// void XRanging_P::init ( void ) {
//   isXLaserInit [ LEFT ]      = true;
//   triggerThreshold [ LEFT ]  = 200;
//   isXLaserInit [ RIGHT ]     = true;
//   triggerThreshold [ RIGHT ] = 200;
//   isXLaserInit [ FRONT ]     = true;
//   triggerThreshold [ FRONT ] = 200;
//   isXLaserInit [ BACK ]      = true;
//   triggerThreshold [ BACK ]  = 200;
// }

// void XRanging_P::init ( laser_e laser, int16_t threshold ) {
//   isXLaserInit [ laser ]     = true;
//   triggerThreshold [ laser ] = threshold;
// }

// /* ============================================================
//  * Read distance from a laser
//  * - Returns -1 if laser is disabled
//  * ============================================================ */
// int16_t XRanging_P::getRange ( laser_e laser ) {

//   if ( ! isXLaserInit [ laser ] ) return -1;

//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     if ( lasers [ i ].id == laser ) {
//       return lasers [ i ].sensor->startRanging ( );
//     }
//   }

//   return -1;
// }

// /* ============================================================
//  * Trigger logic
//  * - Compares range against configured threshold
//  * - Updates LED state
//  * ============================================================ */
// bool XRanging_P::isTriggered ( laser_e laser ) {

//   if ( ! isXLaserInit [ laser ] ) return false;
//   if ( triggerThreshold [ laser ] < 0 ) return false;

//   int16_t range = getRange ( laser );
//   if ( range < 0 ) return false;

//   bool triggered = ( range < triggerThreshold [ laser ] );

//   if ( isValidGpio ( lasers [ laser ].ledGpio ) ) {
//     Peripheral_Write ( lasers [ laser ].ledGpio, triggered ? STATE_HIGH : STATE_LOW );
//   }

//   return triggered;
// }

// /* ============================================================
//  * Singleton instance
//  * ============================================================ */
// XRanging_P XRanging;

// #define RC_MID                1500
// #define RC_MIN                1000
// #define RC_MAX                2000

// #define AVOID_RC_MAX          50    // max RC offset

// #define OA_MAX_PUSH_TIME_MS   500    // max time OA is allowed to push
// #define OA_CLEAR_HOLD_MS      300
// #define OA_BRAKE_GAIN         0.35f

// #define USER_OA_BLEND_PUSH    0.3f    // user authority during OA push
// #define USER_OA_BLEND_BRAKE   0.2f    // even less during braking

// #define AVOIDANCE_INTERVAL_MS 100

// static uint16_t _objectAoidDistance = 0;
// static uint32_t lastAvoidanceRunMs  = 0;

// static int16_t filterDistance ( uint8_t idx, int16_t d ) {
//   static int16_t dFilt [ 5 ] = { 0 };

//   if ( d <= 0 ) return dFilt [ idx ];
//   dFilt [ idx ] = ( dFilt [ idx ] * 3 + d ) / 4;
//   return dFilt [ idx ];
// }

// static int16_t mapAvoidance ( int16_t distance, bool invert ) {

//   if ( distance <= 0 )
//     return 0;

//   // Full OA as soon as object is within 400 mm
//   if ( distance > _objectAoidDistance )
//     return 0;

//   int16_t rcOffset = AVOID_RC_MAX;

//   if ( invert )
//     rcOffset = -rcOffset;

//   return rcOffset;
// }

// bool _enOA = false;

// void enableOA ( ) {
//   _enOA = true;
// }
// void disableOA ( ) {
//   _enOA = false;
//   for ( uint8_t i = 0; i < LASER_COUNT; i++ ) {
//     if ( ! isXLaserInit [ lasers [ i ].id ] ) continue;
//     if ( isValidGpio ( lasers [ i ].ledGpio ) ) {
//       Peripheral_Write ( lasers [ i ].ledGpio, STATE_LOW );
//     }
//   }
// }

// void XRanging_P::initObjectAvoidanceBraking ( uint16_t _avoidDist, laser_e _laser1, laser_e _laser2, laser_e _laser3, laser_e _laser4 ) {
//   // Helper macro to enable a laser safely
//   auto enableLaser = [ & ] ( laser_e l ) {
//     if ( l < 0 )
//       return;

//     isXLaserInit [ l ] = true;
//   };
//   _objectAoidDistance = _avoidDist;
//   // Enable only requested lasers
//   enableLaser ( _laser1 );
//   enableLaser ( _laser2 );
//   enableLaser ( _laser3 );
//   enableLaser ( _laser4 );
// }

// void applyObjectAvoidance ( void ) {

//   uint32_t now = millis ( );
//   if ( now - lastAvoidanceRunMs < AVOIDANCE_INTERVAL_MS )
//     return;
//   lastAvoidanceRunMs = now;

//   if ( ! _enOA )
//     return;

//   /* ---------------- User RC (centered) ---------------- */
//   int16_t userRoll  = rcData [ ROLL ] - 1500;
//   int16_t userPitch = rcData [ PITCH ] - 1500;

//   int16_t rollOffset  = 0;
//   int16_t pitchOffset = 0;

//   bool rollSense  = false;
//   bool pitchSense = false;

//   /* ---------------- Sensors ---------------- */
//   if ( isXLaserInit [ LEFT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 0, XRanging.getRange ( LEFT ) ), false );
//     if ( o ) rollSense = true;
//     Peripheral_Write ( lasers [ LEFT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     rollOffset += o;
//   }

//   if ( isXLaserInit [ RIGHT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 1, XRanging.getRange ( RIGHT ) ), true );
//     if ( o ) rollSense = true;
//     Peripheral_Write ( lasers [ RIGHT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     rollOffset += o;
//   }

//   if ( isXLaserInit [ FRONT ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 2, XRanging.getRange ( FRONT ) ), true );
//     if ( o ) pitchSense = true;
//     Peripheral_Write ( lasers [ FRONT ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     pitchOffset += o;
//   }

//   if ( isXLaserInit [ BACK ] ) {
//     int16_t o = mapAvoidance (
//               filterDistance ( 3, XRanging.getRange ( BACK ) ), false );
//     if ( o ) pitchSense = true;
//     Peripheral_Write ( lasers [ BACK ].ledGpio, o ? STATE_HIGH : STATE_LOW );
//     pitchOffset += o;
//   }

//   /* ---------------- Latch OA timing & force ---------------- */
//   static uint32_t rollOAStartTs  = 0;
//   static uint32_t pitchOAStartTs = 0;
//   static int16_t lastRollOA      = 0;
//   static int16_t lastPitchOA     = 0;

//   if ( rollSense ) {
//     if ( rollOAStartTs == 0 ) rollOAStartTs = now;
//     lastRollOA = rollOffset;
//   }

//   if ( pitchSense ) {
//     if ( pitchOAStartTs == 0 ) pitchOAStartTs = now;
//     lastPitchOA = pitchOffset;
//   }

//   /* ========================== ROLL ========================== */
//   if ( rollOAStartTs != 0 && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS ) ) {

//     // PUSH phase
//     RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_PUSH ) + rollOffset;
//   } else if ( rollOAStartTs != 0 && ( now - rollOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) ) {

//     // BRAKE phase
//     RC_ARRAY [ ROLL ] = ( userRoll * USER_OA_BLEND_BRAKE ) + ( -lastRollOA * OA_BRAKE_GAIN );
//   } else {
//     // OA finished
//     rollOAStartTs     = 0;
//     lastRollOA        = 0;
//     RC_ARRAY [ ROLL ] = userRoll;
//   }

//   /* ========================== PITCH ========================== */
//   if ( pitchOAStartTs != 0 && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS ) ) {

//     // PUSH phase
//     RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_PUSH ) + pitchOffset;
//   } else if ( pitchOAStartTs != 0 && ( now - pitchOAStartTs < OA_MAX_PUSH_TIME_MS + OA_CLEAR_HOLD_MS ) ) {

//     // BRAKE phase
//     RC_ARRAY [ PITCH ] = ( userPitch * USER_OA_BLEND_BRAKE ) + ( -lastPitchOA * OA_BRAKE_GAIN );
//   } else {
//     // OA finished
//     pitchOAStartTs     = 0;
//     lastPitchOA        = 0;
//     RC_ARRAY [ PITCH ] = userPitch;
//   }

//   /* ---------------- Clamp & commit ---------------- */
//   RC_ARRAY [ ROLL ]  = constrain ( RC_ARRAY [ ROLL ], -500, 500 );
//   RC_ARRAY [ PITCH ] = constrain ( RC_ARRAY [ PITCH ], -500, 500 );

//   userRCflag [ ROLL ]  = true;
//   userRCflag [ PITCH ] = true;
// }
