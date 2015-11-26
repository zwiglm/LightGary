#include <Encoder.h>

const int MetalCLK = 2; // Used for generating interrupts using CLK signal
const int MetalDT = 4; // Used for reading DT signal
const int MetalSwitch = 8; // Used for the push button switch

long metalPosition = 0;

Encoder metalEncoder(MetalCLK, MetalDT);


void setup () {
  pinMode(MetalSwitch, INPUT);
  Serial.begin (9600);
  Serial.println("START");
}

void loop () {

    long metalRead = metalEncoder.read();
    
    if (!digitalRead(MetalSwitch)) {  // check if pushbutton is pressed
      metalEncoder.write(0);    
      while (!digitalRead(MetalSwitch)) {} 
      Serial.println("Reset to zero");
    }
    
    if (metalPosition != metalRead) {
      metalPosition = metalRead;
      Serial.print("Count: ");
      Serial.println(metalPosition);
    }
  
}
