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
static Interval blinkTimer;
static int blinksRemaining = 0;
static bool ledOn = false;
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_PPM );    // PPM based
}
static int times;

void startBlinkSequence(int blinkCount) {
  // Each blink has an ON and OFF transition.
  blinksRemaining = blinkCount * 2;
  ledOn = false;
  Set_LED(RED, OFF);
  blinkTimer.reset();
}

void blinkRedCount(int blinkCount) {
  startBlinkSequence(blinkCount);

  while (blinksRemaining > 0) {
    if (blinkTimer.set(300, true)) {
      ledOn = !ledOn;
      Set_LED(RED, ledOn ? ON : OFF);
      blinksRemaining--;
    }
  }

  Set_LED(RED, OFF);
}

void onLoopStartAutoInsertion(){
    Set_LED(STATUS, OFF);
}

void onLoopStopAutoInsertion(){
    Set_LED(STATUS, ON);
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  setUserLoopFrequency(100);
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // do your one time stuffs here
  onLoopStartAutoInsertion();
  Set_LED(RED, OFF);
  Set_LED(BLUE, OFF);
  Set_LED(GREEN, OFF);
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  // Pitch is returned in deciDegrees, so 3600 means one full turn.
  int x = Estimate_Get(Angle, AG_PITCH);
  // Yaw is returned in plain degrees, so 360 means one full turn.
  int y = Estimate_Get(Angle, AG_YAW);

  int x1 = x / 3600;
  int y1 = y / 360;

  // This stores the current whole-turn count derived from pitch and yaw.
  times = x1 + y1;
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // Show the stored turn count after the flight loop stops.
  if (times > 0) {
    blinkRedCount(times);
  }
}
