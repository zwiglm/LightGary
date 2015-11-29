#include <Encoder.h>

const int _metalCLK = 2; // Used for generating interrupts using CLK signal
const int _metalDT = 4; // Used for reading DT signal
const int _metalSwitch = 8; // Used for the push button switch

Encoder metalEncoder(_metalCLK, _metalDT);
long metalPosition = 0;

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

unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 75;
int numRegisters = 1;
int numRGBleds = 1;


// ---------------------------------
void setup () {
  pinMode(_metalSwitch, INPUT);

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

    // Turn all LED's off.
    ShiftPWM.SetAll(0);
  
    // Print information about the interrupt frequency, duration and load on your program
    ShiftPWM.PrintInterruptLoad();
  
    // Fade in and fade out all outputs one by one fast. Usefull for testing your hardware. Use OneByOneSlow when this is going to fast.
    ShiftPWM.OneByOneFast();
  
    // Fade in all outputs
    for(int j=0;j<maxBrightness;j++){
      ShiftPWM.SetAll(j);  
      delay(20);
    }
    // Fade out all outputs
    for(int j=maxBrightness;j>=0;j--){
      ShiftPWM.SetAll(j);  
      delay(20);
    }


    long metalRead = metalEncoder.read();
    
    if (!digitalRead(_metalSwitch)) {  // check if pushbutton is pressed
      metalEncoder.write(0);    
      while (!digitalRead(_metalSwitch)) {} 
      Serial.println("Reset to zero");
    }
    
    if (metalPosition != metalRead) {
      metalPosition = metalRead;
      Serial.print("Metal count: ");
      Serial.println(metalPosition);
    }
  
}
