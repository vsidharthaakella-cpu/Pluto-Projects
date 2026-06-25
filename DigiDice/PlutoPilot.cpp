// Do not remove the include below
#include "PlutoPilot.h"
#include <math.h>
#include <stdlib.h>

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
static float previousNetAcc = 0.0f;

void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_PPM );    // PPM based
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
  onLoopStartAutoInsertion();
  Set_LED(RED, OFF);
  Set_LED(BLUE, OFF);
  Set_LED(GREEN, OFF);
  
}
//this function allows me to use the timer function
void startBlinkSequence(void) {
  int blinkCount = (rand() % 6) + 1;// the amount of blinks
  blinksRemaining = blinkCount * 2;  // on + off transitions
  ledOn = false;// this is to see if it repeats
  Set_LED(RED, OFF);// turns off to begin with
  blinkTimer.reset();// resets the timer
} 

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  // Add your repeated code here
  float ax = Sensor_Get(Accelerometer, X);
  float ay = Sensor_Get(Accelerometer, Y);
  float az = Sensor_Get(Accelerometer, Z);

  float Net_Acc = sqrtf(ax*ax + ay*ay + az*az);
  float deltaAcc = fabsf(Net_Acc - previousNetAcc);// this checks how much the acceleration changed from before
  previousNetAcc = Net_Acc;// this stores the current value so i can compare it in the next loop

  if(deltaAcc > 200 && blinksRemaining == 0){// this checks for a shake and makes sure a new blink starts only when the old one is done
    startBlinkSequence();// this starts the random blink sequence after the drone is shaken
  }

  if (blinksRemaining > 0 && blinkTimer.set(300, true)) {// allows me to check whether the blinks a static variable is equal to zero. if not and the timer is set to default
    ledOn = !ledOn;// this changes the intial condition to be opposite, of what it was before, that way there is an alternating action 
    Set_LED(RED, ledOn ? ON : OFF);// the LED will now turn on or off as a mimic to whether LedOn is true Or false.
    blinksRemaining--; // this is a subtraction of the number of blinks present, technically mimicking the dice roll. 
  }

}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  onLoopStopAutoInsertion();

}
