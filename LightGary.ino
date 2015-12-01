#include <PinChangeInt.h>
#include <Encoder.h>

const int _metalSwitch = 8; // Used for the push button switch

const int _metalIRQ = 0;
const int _metalCLK = 2; // Used for generating interrupts using CLK signal
const int _metalDT = 4; // Used for reading DT signal
const int _rgbReIRQ = 1;
const int _rgbReCLK = 3; // Used for generating interrupts using CLK signal
const int _rgbReDT = 6; // Used for reading DT signal

Encoder metalEncoder(_metalCLK, _metalDT);
Encoder rgbEncoder(_rgbReCLK, _rgbReDT);
volatile int  metalPosition = 0;
volatile int  rgbPosition = 0;

//-----------------------------------
#define SHIFTPWM_USE_TIMER2  // for Arduino Uno and earlier (Atmega328)
const int ShiftPWM_latchPin = 7;
#define SHIFTPWM_NOSPI
const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;

// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = false; 

// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

unsigned char maxBrightness = 50; //255
unsigned char pwmFrequency = 75;
int numRegisters = 1;
int numRGBleds = 1;


// ---------------------------------
void setup () {
  pinMode(_metalSwitch, INPUT);
  attachPinChangeInterrupt(_metalSwitch, metalSwitchWakeup, RISING);

  pinMode(_metalCLK, INPUT);
  pinMode(_metalDT, INPUT); 
  attachInterrupt(_metalIRQ, metalEncWakeup, CHANGE);
  pinMode(_rgbReCLK, INPUT);
  pinMode(_rgbReDT, INPUT); 
  attachInterrupt(_rgbReIRQ, rgbEncWakeup, CHANGE);
  
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  // SetPinGrouping allows flexibility in LED setup. 
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion  
  ShiftPWM.Start(pwmFrequency, maxBrightness);

  Serial.begin (9600);
  Serial.println("START");
}

void loop () {

//    // Turn all LED's off.
//    ShiftPWM.SetAll(0);
//  
//    // Print information about the interrupt frequency, duration and load on your program
////    ShiftPWM.PrintInterruptLoad();
//  
//    // Fade in and fade out all outputs one by one fast. Usefull for testing your hardware. Use OneByOneSlow when this is going to fast.
//    ShiftPWM.OneByOneFast();
//  
//    // Fade in all outputs
//    for(int j=0;j<maxBrightness;j++){
//      ShiftPWM.SetAll(j);  
//      delay(20);
//    }
//    // Fade out all outputs
//    for(int j=maxBrightness;j>=0;j--){
//      ShiftPWM.SetAll(j);  
//      delay(20);
//    }

    int lastMetalCount = 0;
    int lastRgbCount = 0;
    while (true) {
        if (metalPosition != lastMetalCount) {
          lastMetalCount = metalPosition;
          Serial.print("Count metal:");
          Serial.println(metalPosition);
        }
        if (rgbPosition != lastRgbCount) {
          lastRgbCount = rgbPosition;
          Serial.print("Count RGB:");
          Serial.println(rgbPosition);
        }
    } // while  
}


// ---------------------------------
void metalSwitchWakeup() {
  detachPinChangeInterrupt(_metalSwitch);
  metalPosition = 0;    
  attachPinChangeInterrupt(_metalSwitch, metalEncWakeup, FALLING);
}

void metalEncWakeup() {
  detachInterrupt(_metalIRQ);
  
    /* If pinA and pinB are both high or both low, it is spinning
     * forward. If they're different, it's going backward.
     *
     * For more information on speeding up this process, see
     * [Reference/PortManipulation], specifically the PIND register.
     */
    if (digitalRead(_metalCLK) == digitalRead(_metalDT)) {
      metalPosition--;
    } else {
      metalPosition++;
    }
  
  attachInterrupt(_metalIRQ, metalEncWakeup, CHANGE);
}

void rgbEncWakeup() {
  detachInterrupt(_rgbReIRQ);
  
    if (digitalRead(_rgbReCLK) == digitalRead(_rgbReDT)) {
      rgbPosition--;
    } else {
      rgbPosition++;
    }
  
  attachInterrupt(_rgbReIRQ, rgbEncWakeup, CHANGE);
}




// ---------------------------------

//void metalEncWakeup_Expanded(){
//  if (digitalRead(_metalCLK) == HIGH) {   // found a low-to-high on channel A
//    if (digitalRead(_metalDT) == LOW) {  // check channel B to see which way
//                                             // encoder is turning
//      metalPosition = metalPosition - 1;         // CCW
//    } 
//    else {
//      metalPosition = metalPosition + 1;         // CW
//    }
//  }
//  else                                        // found a high-to-low on channel A
//  { 
//    if (digitalRead(_metalDT) == LOW) {   // check channel B to see which way
//                                              // encoder is turning  
//      metalPosition = metalPosition + 1;          // CW
//    } 
//    else {
//      metalPosition = metalPosition - 1;          // CCW
//    }
//
//  }
//}



