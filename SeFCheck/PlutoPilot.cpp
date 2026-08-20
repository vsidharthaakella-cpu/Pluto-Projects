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
void onLoopStartAutoInsertion(){
    Set_LED(STATUS, OFF);
}
void onLoopStopAutoInsertion(){
    Set_LED(STATUS, ON);
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  //XRanging.init(FRONT,50);//intialized all values
  XRanging.init(BACK,50);
  // XRanging.init(RIGHT, 50);
  // XRanging.init(LEFT,50);
  setUserLoopFrequency(1000);

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
  // Add your repeated code here
  Set_LED(RED, OFF);
  Set_LED(GREEN, OFF);
  Set_LED(BLUE, OFF);

  // if(XRanging.isTriggered(FRONT)){
  //   int front = XRanging.getRange(FRONT);
  //   if(front == -100){
  //     Set_LED(RED, ON);
  //   }
  //   else{
  //     Set_LED(GREEN, ON);
  //   }
  // }
    if(XRanging.isTriggered(LEFT)){
      int left = XRanging.getRange(LEFT);
      if(left == -100){
        Set_LED(RED, ON);
    }
      else{
        Set_LED(GREEN, ON);
    }
  }
  // else if(XRanging.isTriggered(RIGHT)){
  //   int right = XRanging.getRange(RIGHT);
  //   if(right == -100){
  //     Set_LED(RED, ON);
  //   }
  //   else{
  //     Set_LED(GREEN, ON);
  //   }
  // }
  // else if(XRanging.isTriggered(BACK)){
  //   int back = XRanging.getRange(BACK);
  //   if(back == -100){
  //     Set_LED(RED, ON);
  //   }
  //   else{
  //     Set_LED(GREEN, ON);
  //   }
  
  // }
  else{
    Set_LED(BLUE, ON);
  }
}


// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // do your cleanup stuffs here
  onLoopStopAutoInsertion();
}
