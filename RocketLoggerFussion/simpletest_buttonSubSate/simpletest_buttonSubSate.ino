/*

Description:
  -State 0: 
            LED off
  -State 1: 
            LED half-on
  -State 2: 
            LED full-on

  -If the button is pressed for longer than 2 seconds 'cancel' the change in state

*/

//////////////////////////// INITIALIZE PUBLIC VARIABLES  /////////////////////////////
const int switchPin = 15;                      // switch monitors pin 14
const int Red = 6;                            // the Red led is on pin 15

int buttonState;                              // remember the state of the button
int State = 0;                                // turns the led on and off during blinking
                                                  // 0 = off
                                                  // 1 = state1
                                                  // 2 = state2

unsigned long buttonPrevious = 0; // [ms]     // store the time when the button was initially pressed
const int buttonInterval = 1000; // [ms]      // interval for button being pressed to 'cancel' the change in state
unsigned long buttonDiff = 0; // [ms]         // store the difference in time between button press and release

unsigned long startState0 = 0; // [ms]
unsigned long startState1 = 0; // [ms]
unsigned long startState2 = 0; // [ms]

void setup() {
  pinMode(switchPin, INPUT);                  // set the switch pin as the input 
  pinMode(Red, OUTPUT);                       // set the led pin as the output
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
        if (State == 0) {                // check if the light was previously in state 0
          State = 1;                     // change the light state to 1
          Serial.println("Button just pressed, State 1");
        }else if (State == 1){           // check if the light was previously in state 1 
          State = 2;                     // change the light state to 2
          Serial.println("Button just pressed, State 2");
        }else{                                // check if the light was previously in state 2 
          State = 0;                     // change the light state to 0
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

void State0(){                                // execute state 0
  if(State != 0){
    startState0 = millis();
  }else if(State == 0){
    analogWrite(Red, 0);
  }
}

void State1(){                                // execute state 1
  if(State != 1){
    startState1 = millis();
  }else if (State == 1) {                  
    analogWrite(Red, 100);
  }
}

void State2(){                                // execute state 2
  if(State != 2){
    startState2 = millis();
  }else if (State == 2) {
    analogWrite(Red, 255);
  }
}
