#include <Wire.h>
#include <RTClib.h>
#include <PinChangeInt.h>
#include <Encoder.h>

//-----------------------------------
RTC_DS1307 _rtc;
const int _sSegLatch = A2;
const int _sSegClock = A1;
const int _sSegData = A3;

byte _sSegControl[5] = { B11111000, B00111100, B01011100, B01101100, B01110100 };  // 1 .. 8 -> 6 LED for doppel-punkt; 7 & 8 most front dots; 1 -> doppel-Punkt; 2-> 1st dig, 3-> 2nd dig; 4 -> 4th dig; 6 -> 3rd dig
byte _sSegDigits[10] = { B11111010, B00110000, B11011100, B01111100, B00110110, B01101110, B11101110, B00111000, B11111110, B01111110 };
 
//-----------------------------------
const int _metalSwitch = 8; // Used for the push button switch
const int _rgbReSwitch = 9;

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
const bool ShiftPWM_invertOutputs = true; 

// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

unsigned char maxBrightness = 50; //255
unsigned char pwmFrequency = 75;
int numRegisters = 1;
int numRGBleds = 2;


// ---------------------------------
void setup () {
  Serial.begin (9600);
  Serial.println("START");

  // RTC
  Wire.begin();
  _rtc.begin();
  if (!_rtc.isrunning()) {
    Serial.println("RTC is NOT running");
    _rtc.adjust(DateTime(__DATE__, __TIME__));
  }

  // 7 - Segment
  pinMode(_sSegLatch, OUTPUT);
  pinMode(_sSegData, OUTPUT);
  pinMode(_sSegClock, OUTPUT);

  // Rotary Encoders
  pinMode(_metalSwitch, INPUT);
  attachPinChangeInterrupt(_metalSwitch, metalSwitchWakeup, RISING);
  pinMode(_rgbReSwitch, INPUT);
  attachPinChangeInterrupt(_rgbReSwitch, rgbReSwitchWakeup, FALLING);

  pinMode(_metalCLK, INPUT);
  pinMode(_metalDT, INPUT); 
  attachInterrupt(_metalIRQ, metalEncWakeup, CHANGE);
  pinMode(_rgbReCLK, INPUT);
  pinMode(_rgbReDT, INPUT); 
  attachInterrupt(_rgbReIRQ, rgbEncWakeup, CHANGE);

  // FIRST Shift-Register
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  // SetPinGrouping allows flexibility in LED setup. 
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion  
  ShiftPWM.Start(pwmFrequency, maxBrightness);
}

void loop () {
    // Time
    showTimeOn7Seg();
    
    // Turn all LED's off.
    ShiftPWM.SetAll(0);  
    // Print information about the interrupt frequency, duration and load on your program
//    ShiftPWM.PrintInterruptLoad();  
    // Fade in and fade out all outputs one by one fast. Usefull for testing your hardware. Use OneByOneSlow when this is going to fast.
//    ShiftPWM.OneByOneSlow();


    ShiftPWM.SetRGB(0, 0, 0, 255);
    ShiftPWM.SetRGB(1, 0, 255, 0);
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

//    int lastMetalCount = 0;
//    int lastRgbCount = 0;
//    while (true) {
//        if (metalPosition != lastMetalCount) {
//          lastMetalCount = metalPosition;
//          Serial.print("Count metal:");
//          Serial.println(metalPosition);
//        }
//        if (rgbPosition != lastRgbCount) {
//          lastRgbCount = rgbPosition;
//          Serial.print("Count RGB:");
//          Serial.println(rgbPosition);
//        }
//    } // while  
}


// ---------------------------------
void showTimeOn7Seg() {
  DateTime now = _rtc.now();
  uint8_t lHour = now.hour();
  uint8_t lMin = now.minute();

//  Serial.print(lHour, DEC);
//  Serial.print(':');
//  Serial.print(lMin, DEC);
//  Serial.print(':');
//  Serial.print(now.second(), DEC);
//  Serial.println();

  // last part of min
  uint8_t digit = lMin % 10;
  writeSevenSegment(_sSegControl[4], _sSegDigits[digit]);
  delay(3);
  // first part of min
  digit = lMin / 10;
  writeSevenSegment(_sSegControl[3], _sSegDigits[digit]);
  delay(3);

  // last part of hour
  digit = lHour % 10;
  writeSevenSegment(_sSegControl[2], _sSegDigits[digit]);
  delay(3);
  // firstr part of hour
  digit = lHour / 10;
  writeSevenSegment(_sSegControl[1], _sSegDigits[digit]);
  delay(3);
}
void writeSevenSegment(byte stShift, byte ndShift)
{
  // turn off the output so the pins don't light up
  // while you're shifting bits:
  digitalWrite(_sSegLatch, LOW);
  
  // shift the bits out:
  shiftOut(_sSegData, _sSegClock, LSBFIRST, ndShift);
  shiftOut(_sSegData, _sSegClock, LSBFIRST, stShift);
  
  // turn on the output so the LEDs can light up:
  digitalWrite(_sSegLatch, HIGH);
}


// ---------------------------------
void metalSwitchWakeup() {
  detachPinChangeInterrupt(_metalSwitch);
  metalPosition = 0;    
  attachPinChangeInterrupt(_metalSwitch, metalSwitchWakeup, RISING);
}

void rgbReSwitchWakeup() {
  detachPinChangeInterrupt(_rgbReSwitch);
  rgbPosition = 0;    
  attachPinChangeInterrupt(_rgbReSwitch, rgbReSwitchWakeup, FALLING);
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



