/*

Hato Ando
Samuel Newport
03 October 2024
MAE 311L - 06
Mini Project: Main

Description:
  -State 0: 
            PWM RGB values set to 7
  -State 1: 
            Blinking Pattern
  -State 2: 
            Exponential Fade Pattern

  -If the button is pressed for longer than 2 seconds 'cancel' the change in state

*/

#include <math.h>

//////////////////////////// INITIALIZE PUBLIC VARIABLES  /////////////////////////////
const int switchPin = 14;                      // switch monitors pin 2
const int Red = 6;                            // the Red led is on pin 6
const int Green = 7;                         // the green led is on pin 7
const int Blue = 8;                          // the blue led is on pin 8

int buttonState;                              // remember the state of the button
int lightState = 0;                           // turns the led on and off during blinking
                                                  // 0 = off
                                                  // 1 = state1
                                                  // 2 = state2

unsigned long buttonPrevious = 0; // [ms]     // store the time when the button was initially pressed
const int buttonInterval = 1000; // [ms]      // interval for button being pressed to 'cancel' the change in state
unsigned long buttonDiff = 0; // [ms]         // store the difference in time between button press and release

unsigned long startState1 = 0;
unsigned long startState2 = 0;

int I = 255;

float t0;
float t1;
float t2;
int subState = 0;

void setup() {
  pinMode(switchPin, INPUT);                  // set the switch pin as the input 
  pinMode(Red, OUTPUT);                       // set the led pin as the output
  pinMode(Green, OUTPUT);
  pinMode(Blue, OUTPUT);
  Serial.begin(9600);                         // set up the serial communication
  while(!Serial){}
  buttonState = digitalRead(switchPin);       // read the initial button state
  Serial.println();
  Serial.println("System start up, State 0");
}

void loop() {
  int val = digitalRead(switchPin);           // read the input of the switch (1/0)
  delay(10);                                  // wait 10 ms to read switch state again
  int val2 = digitalRead(switchPin);          // read the input of the switch (1/0)

  if (val == val2) {                          // compare the 2 readings for bounce
    buttonDuration(val);                      // check the button press duration
    if (val != buttonState && buttonDiff < buttonInterval) { // check if the state has changed AND if the button has been pressed longer than the interval value
      if (val == LOW) {                       // check if the button is being pressed
        if (lightState == 0) {                // check if the light was previously in state 0
          lightState = 1;                     // change the light state to 1
          Serial.println("Button just pressed, State 1");
        }else if (lightState == 1){           // check if the light was previously in state 1 
          lightState = 2;                     // change the light state to 2
          Serial.println("Button just pressed, State 2");
        }else{                                // check if the light was previously in state 2 
          lightState = 0;                     // change the light state to 0
          Serial.println("Button just pressed, State 0");
        }
      }
    }
  }
  //////////////////////////// EXECUTE STATE FUNCTIONS  /////////////////////////
  State0();
  State1();
  State2();
  buttonState = val;                          // update the button state
}

long buttonDuration(int val){                 // checking button press duration
  if(val != 1){                               // if the button has been pressed and has passed the debounce filter
    buttonPrevious = millis();                // if the button has NOT been pressed keep the previous button time updated
  }else{
    buttonDiff = millis() - buttonPrevious;   // if the button HAS been pressed log the duration of the press
  }
  return buttonDiff;
}

void State0(){                                 // execute state 0
  if(lightState == 0){
    analogWrite(Red, 7);
    analogWrite(Green, 7);
    analogWrite(Blue, 7);
  }
}

void State1(){                                // execute state 1
  unsigned long tdelay1 = 1000;
  unsigned long tdelay2 = 350;
  unsigned long tdelay3 = 150;
  unsigned long tdelay4 = 1000;

  if(lightState != 1){
    startState1 = millis();
  }else if (lightState == 1) {                  // check if the light is ON
    if(millis()-startState1 <= tdelay1){
      analogWrite(Red, 100);
      analogWrite(Green, 0);
      analogWrite(Blue, 0);
    }else if(millis()-startState1 <= tdelay2+tdelay1){
      analogWrite(Red, 255);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);
    }else if(millis()-startState1 <= tdelay3+tdelay2+tdelay1){
      analogWrite(Red, 0);
      analogWrite(Green, 0);
      analogWrite(Blue, 0);
    }else if(millis()-startState1 <= tdelay3+2*tdelay2+tdelay1){
      analogWrite(Red, 255);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);
    }else if(millis()-startState1 <= 2*tdelay3+2*tdelay2+tdelay1){
      analogWrite(Red, 0);
      analogWrite(Green, 0);
      analogWrite(Blue, 0);
    }else if(millis()-startState1 <= tdelay4+2*tdelay3+2*tdelay2+tdelay1){
      analogWrite(Red, 0);
      analogWrite(Green, 0);
      analogWrite(Blue, 100);      
    }else{
      startState1 = millis();
    }
  }
}

void State2(){                                // execute state 2
  unsigned long tdelay = 1000;
  if(lightState != 2){
    startState2 = millis();
    t0 = millis();
    t1 = millis();
    t2 = millis();
  }else if (lightState == 2) {
    if(millis() - startState2 <= tdelay){
      subState = 0;
      t1 = millis();
      t2 = millis();
    }else if(millis() - startState2 <= 2*tdelay){
      subState = 1;
      t0 = millis();
      t2 = millis();
    }else if(millis() - startState2 <= 3*tdelay){
      subState = 2;
      t0 = millis();
      t1 = millis();
    }else{
      startState2 = millis();
      t0 = millis();
      t1 = millis();
      t2 = millis();
    }
    
    if(subState == 0){
      analogWrite(Red, Intensity(millis()-t0));
      analogWrite(Green, 0);
      analogWrite(Blue, 0);
    }else if(subState == 1){
      analogWrite(Red, Intensity(millis()-t1));
      analogWrite(Green, Intensity(millis()-t1));
      analogWrite(Blue, 0);    
    }else{
      analogWrite(Red, Intensity(millis()-t2));
      analogWrite(Green, Intensity(millis()-t2));
      analogWrite(Blue, Intensity(millis()-t2));
    }
  }
}

int Intensity(int t){                         // calculate LED intensity
  I = pow(((980-t)/40),1.732);
  return I;
}