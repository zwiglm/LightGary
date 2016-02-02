#include <Wire.h>
#include <RTClib.h>
#include <PinChangeInt.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>

//-----------------------------------
// Neopixel
#define NP_PIN A0
Adafruit_NeoPixel _npRing = Adafruit_NeoPixel(24, NP_PIN, NEO_GRB + NEO_KHZ800);

//-----------------------------------
// RealTimeClock
RTC_DS1307 _rtc;
const int _sSegLatch = A2;
const int _sSegClock = A1;
const int _sSegData = A3;

byte _sSegControl[5] = { B11111000, B00111100, B01011100, B01101100, B01110100 };  // 1 .. 8 -> 6 LED for doppel-punkt; 7 & 8 most front dots; 1 -> doppel-Punkt; 2-> 1st dig, 3-> 2nd dig; 4 -> 4th dig; 6 -> 3rd dig
byte _sSegDigits[10] = { B11111010, B00110000, B11011100, B01111100, B00110110, B01101110, B11101110, B00111000, B11111110, B01111110 };

//-----------------------------------
// Rotary Encoders with Button
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

volatile int  _metalPosition = 0;
volatile int  _rgbPosition = 0;

volatile int _metalButton = 0;
const int _maxMetalButton = 3;
volatile int _rgbButton = 0;
const int _maxRgbButton = 8;

uint32_t _aRgbButton[_maxRgbButton] = { 1, 2, 4, 8, 16, 32, 64, 128 };
uint32_t _aMetalButton[_maxMetalButton] = { 1024, 2048, 4096 };

//-----------------------------------
// Setup for ShiftRegister PWM-Lib
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
  
    // NeoPixel Ring
    _npRing.begin();
    _npRing.show(); // Initialize all pixels to 'off'
    
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
  
  
    // DO some Hardware-Testing
    // NP Ring test
//    neoColorWipe(_npRing.Color(255, 0, 0), 50); // Red
//    neoColorWipe(_npRing.Color(0, 255, 0), 50); // Green
//    neoColorWipe(_npRing.Color(0, 0, 255), 50); // Blue
//    neoClearAll();

    // Turn all LED's off.
    ShiftPWM.SetAll(0);  
//    // Print information about the interrupt frequency, duration and load on your program
//    ShiftPWM.PrintInterruptLoad();  
//    // Fade in and fade out all outputs one by one fast. Usefull for testing your hardware. Use OneByOneSlow when this is going to fast.
//    ShiftPWM.OneByOneSlow();
//
//    ShiftPWM.SetRGB(0, 0, 0, 255);
//    ShiftPWM.SetRGB(1, 0, 255, 0);
//    delay(1000);
//    ShiftPWM.SetAll(0);  

    clear7Seg();
}

void loop () {

    uint32_t buttonValue = _aRgbButton[_rgbButton] + _aMetalButton[_metalButton];
    Serial.print("Button-Types:");
    Serial.println(buttonValue);
    switch(buttonValue){
        case 2049: 
          // Time
          showTimeOn7Seg();
          break;
          
        default:
          clearAll();
          break;
    }
      
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
//        if (_metalPosition != lastMetalCount) {
//          lastMetalCount = _metalPosition;
//          Serial.print("Count metal:");
//          Serial.println(_metalPosition);
//        }
//        if (_rgbPosition != lastRgbCount) {
//          lastRgbCount = _rgbPosition;
//          Serial.print("Count RGB:");
//          Serial.println(_rgbPosition);
//        }
//    } // while  
}


// ---------------------------------
void clearAll() {
    clear7Seg();
}

// ---------------------------------
// Fill the dots one after the other with a color
void neoColorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i < _npRing.numPixels(); i++) {
    _npRing.setPixelColor(i, c);
    _npRing.show();
    delay(wait);
  }
}
void neoClearAll() {
  uint32_t clearC = _npRing.Color(0, 0, 0);
  for(uint16_t i=0; i < _npRing.numPixels(); i++) {
    _npRing.setPixelColor(i, clearC);
  }
  _npRing.show();
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
void clear7Seg() {
  writeSevenSegment(0, 0);
  writeSevenSegment(0, 0);
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
  
  _metalPosition = 0;
  _metalButton++;
  if (_metalButton > _maxMetalButton - 1)
    _metalButton = 0;
  
  attachPinChangeInterrupt(_metalSwitch, metalSwitchWakeup, RISING);
}

void rgbReSwitchWakeup() {
  detachPinChangeInterrupt(_rgbReSwitch);

  _rgbPosition = 0;
  _rgbButton++;
  if (_rgbButton > _maxRgbButton - 1)
    _rgbButton = 0;
  
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
      _metalPosition--;
    } else {
      _metalPosition++;
    }
  
  attachInterrupt(_metalIRQ, metalEncWakeup, CHANGE);
}

void rgbEncWakeup() {
  detachInterrupt(_rgbReIRQ);
  
    if (digitalRead(_rgbReCLK) == digitalRead(_rgbReDT)) {
      _rgbPosition--;
    } else {
      _rgbPosition++;
    }
  
  attachInterrupt(_rgbReIRQ, rgbEncWakeup, CHANGE);
}




// ---------------------------------
// Just for reference

//void metalEncWakeup_Expanded(){
//  if (digitalRead(_metalCLK) == HIGH) {   // found a low-to-high on channel A
//    if (digitalRead(_metalDT) == LOW) {  // check channel B to see which way
//                                             // encoder is turning
//      _metalPosition = _metalPosition - 1;         // CCW
//    } 
//    else {
//      _metalPosition = _metalPosition + 1;         // CW
//    }
//  }
//  else                                        // found a high-to-low on channel A
//  { 
//    if (digitalRead(_metalDT) == LOW) {   // check channel B to see which way
//                                              // encoder is turning  
//      _metalPosition = _metalPosition + 1;          // CW
//    } 
//    else {
//      _metalPosition = _metalPosition - 1;          // CCW
//    }
//
//  }
//}



