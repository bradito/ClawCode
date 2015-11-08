// Still have problem with grip strength vs average so took Too slow routine out of the averaging...

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"


#define TOTAL_MESSAGES (5)
#define DEBUG (0)
#define X_MOTOR_PORT (1)
#define Y_MOTOR_PORT (2)
#define CLAW_MOTOR_PORT (1)
#define WINCH_MOTOR_PORT (2)
#define X_LEFT_LIMIT (4)
#define X_RIGHT_LIMIT (5)
#define Y_FORWARD_LIMIT (3)
#define Y_BACKWARD_LIMIT (2)
#define WINCH_LIMIT (6)
#define CLAW_BUTTON (7)
#define SCROLL_DELAY (200)
#define WINCH_LOWER_TIME (4000)
#define WINCH_SPEED (100)
#define CLAW_SPEED (140)
#define GAME_TIMER (30)

#define MODE_STARTUP (0)
#define MODE_WAIT_FOR_GAME (1)
#define MODE_POSITION_CLAW (2)
#define MODE_CLAW_DROP (3)
#define MODE_RETURN_HOME (4)

#define WINCH_MODE_RESETTING (0)
#define WINCH_CLAW_RESETTING (1)
#define WINCH_MODE_LOWERING (2)
#define WINCH_CLAW_CLOSING (3)
#define WINCH_MODE_RAISING (4)
#define WINCH_MODE_HOMING (5)
#define WINCH_CLAW_OPEN_AT_HOME (6) 
#define WINCH_MODE_STANDBY (7)

#define X_JOYSTICK_PIN (0)
#define Y_JOYSTICK_PIN (1)
#define CENTER_TRIM (3)
#define CLAW_POSITION (A2)
#define CLAW_TOTAL_OPEN (205)
#define CLAW_TOTAL_CLOSE (175)
#define CLAW_RATE_THRESHOLD (1)
#define CLAW_CHANGE_TIME_LIM (1000)

Adafruit_MotorShield AFMStop(0x60); //top board for stepper X and Y
Adafruit_MotorShield AFMSbot(0x61); //bottom board for winch and claw motor

Adafruit_StepperMotor *XStepper = AFMStop.getStepper(124,X_MOTOR_PORT);
Adafruit_StepperMotor *YStepper = AFMStop.getStepper(124,Y_MOTOR_PORT);

Adafruit_DCMotor *ClawMotor = AFMSbot.getMotor(CLAW_MOTOR_PORT);
Adafruit_DCMotor *WinchMotor = AFMSbot.getMotor(WINCH_MOTOR_PORT);

/// make accel stepper objects

AccelStepper stepperX(forwardstepX, backwardstepX);
AccelStepper stepperY(forwardstepY, backwardstepY);
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

String message0 = " HI - HELLO - WELCOME - HAVE FUN - ";
String message1 = " READY?! Press Button to Play    ";
String message2 = "   GO!";
String message3 = "GRABBING    ";
String message4 = "OOOH DID YOU WIN?!   ";
char nextChar = '-';

String messages[] = {message0,message1,message2,message3,message4};

int messageScrollPosition = 0;
long lastStep = 0;

long winchLowerTimer = 0;

int xJoystickVal = 0;
int xVector = 0;
int yJoystickVal = 0;
int yVector = 0;
int xSteps = 0;
int ySteps = 0;
int overallMode = MODE_STARTUP;
int winchMode = WINCH_MODE_STANDBY;
long gameStartMillis = 0;
int lastTimeElapsed = 0;

int lastClawReading = 0;
int currentClawReading = 0;
int instantReading = 0;
long lastClawChange = 0;

const int numReadings = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

char displaybuffer[4] = {' ', ' ', ' ', ' '};

//wrapper functions

void forwardstepX() {
    XStepper->onestep(FORWARD, SINGLE);
}

void backwardstepX() {
    XStepper->onestep(BACKWARD, SINGLE);
}

// reversed forward for backward for apparent motion found in testing
void forwardstepY() {
    YStepper->onestep(BACKWARD, SINGLE);
}

void backwardstepY() {
    YStepper->onestep(FORWARD, SINGLE);
}

void setup() {
  lastStep=millis(); // set the timer to now.
  AFMStop.begin(); // Start the shield
  alpha4.begin(0x71); //initialize the address of the backpack
  AFMSbot.begin();
  
  stepperX.setMaxSpeed(200.0);
  stepperX.setAcceleration(100.0);
  stepperX.moveTo(24);
  
  stepperY.setMaxSpeed(200.0);
  stepperY.setAcceleration(100.0);
  stepperY.moveTo(24);
  
  if (DEBUG) Serial.begin(9600);
  
  
  // setup limit switches
  pinMode(X_LEFT_LIMIT,INPUT_PULLUP);
  pinMode(X_RIGHT_LIMIT,INPUT_PULLUP);
  pinMode(Y_FORWARD_LIMIT,INPUT_PULLUP);
  pinMode(Y_BACKWARD_LIMIT,INPUT_PULLUP);
  pinMode(WINCH_LIMIT,INPUT_PULLUP);
  pinMode(CLAW_BUTTON,INPUT_PULLUP);
  
  alpha4.clear();
  alpha4.writeDisplay();
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(CLAW_POSITION);
    delay(1);
  }
  
}

void loop() {

  xJoystickVal = analogRead(X_JOYSTICK_PIN);
  yJoystickVal = analogRead(Y_JOYSTICK_PIN);
  xVector = map(xJoystickVal, 0, 1023, -100, 100);
  yVector = map(yJoystickVal, 0, 1023, -100, 100);
 
 //mode management
 switch(overallMode) {
  case MODE_STARTUP:
    if (DEBUG) Serial.println("Case: ModeStartup");
    messageScrollPosition = setNewMessage(MODE_STARTUP);
    if((abs(xVector) < CENTER_TRIM) && (abs(yVector) < CENTER_TRIM)) { // wait for centered joystick to move out of startup
      /// later add brief startup to display the full startup message
      if (DEBUG) Serial.println("Case: Startup Stick Centered");
      overallMode = MODE_WAIT_FOR_GAME;
      messageScrollPosition = setNewMessage(MODE_WAIT_FOR_GAME);
    } else {
      messageScrollPosition = scrollDisplay(MODE_STARTUP, messageScrollPosition);
    }
    break;
  case MODE_WAIT_FOR_GAME:
    if (DEBUG) Serial.println("Case: WaitForGame");
    while(digitalRead(CLAW_BUTTON) == LOW) {
      if (DEBUG) Serial.println("Case: WaitForGame Claw Button Held");
      delay(50); // wait 50 ms to see if it has gone high
      if(digitalRead(CLAW_BUTTON) == HIGH) { // if gone high, then user released the button and so we can switch modes
        if (DEBUG) Serial.println("Case: WaitForGame Claw Button Released");
        overallMode = MODE_POSITION_CLAW;
        messageScrollPosition = setNewMessage(MODE_POSITION_CLAW);
      }
    }
    messageScrollPosition = scrollDisplay(MODE_WAIT_FOR_GAME, messageScrollPosition);
    break;
    
  case MODE_POSITION_CLAW:
    if(gameStartMillis == 0) { // if we are here to start, then start the clock
      gameStartMillis = millis(); // set timer
    } else {
        int timeElapsed = (int) ((millis() - gameStartMillis) / 1000);
        
        if ( timeElapsed > GAME_TIMER)  { // if we've exceeded the timer, advance the mode
        
          if (DEBUG) Serial.println("Case:Timed Out!");
          overallMode = MODE_CLAW_DROP;
          messageScrollPosition = setNewMessage(MODE_CLAW_DROP);
          gameStartMillis = 0;
          
      } else { // within timer
        
          moveXY(xVector, yVector, stepperX, stepperY); // call routine to move xy
          while(digitalRead(CLAW_BUTTON) == LOW) {
            if (DEBUG) Serial.println("Case: PositionClaw Claw Button Held");
            delay(50); // wait 50 ms to see if it has gone high
            if(digitalRead(CLAW_BUTTON) == HIGH) { // if gone high, then user released the button and so we can switch modes
              if (DEBUG) Serial.println("Case: PositionClaw Claw Button Released");
              overallMode = MODE_CLAW_DROP;
              messageScrollPosition = setNewMessage(MODE_CLAW_DROP);
              gameStartMillis = 0;
            }
          }
      }
      
        if(lastTimeElapsed != timeElapsed) {  // if we need to update the display time, update it.
          gameTimerDisplay(GAME_TIMER - timeElapsed); 
          lastTimeElapsed = timeElapsed;
        }

    }
    
    break;
    
  case MODE_CLAW_DROP:

    if(winchMode != WINCH_MODE_STANDBY) { // ok, if we're here and not in standby, then just call runWinch with the last mode was again
       if (DEBUG) Serial.print("Case: ClawDrop, winch mode: ");
       if (DEBUG) Serial.println(winchMode);  
       winchMode = runWinch(WinchMotor, ClawMotor, winchMode);
       if(winchMode == WINCH_MODE_STANDBY) { // but if the new winchMode is now standby, we just completed and should advance the overall mode
        overallMode = MODE_WAIT_FOR_GAME;
        messageScrollPosition = setNewMessage(MODE_WAIT_FOR_GAME);
        if (DEBUG) Serial.println("Case: ClawDrop, end Winch Cycle");
       }
       if (winchMode == WINCH_MODE_HOMING) // if we come back here, we need to advance the overallMode
          overallMode = MODE_RETURN_HOME; 
    } else {
     if (DEBUG) Serial.println("Case: ClawDrop, new Winch Cycle");
      winchMode = runWinch(WinchMotor, ClawMotor, WINCH_MODE_RESETTING); // if we did come here in winch mode standby then we must be at the start of the new cycle, so call with new winch mode to start new cycle 
    }
    messageScrollPosition = scrollDisplay(MODE_CLAW_DROP, messageScrollPosition);
    break;
    
  case MODE_RETURN_HOME:
    if (DEBUG) {
      
      Serial.print("XL:");
      Serial.print(digitalRead(X_LEFT_LIMIT));
      Serial.print("XR:");
      Serial.print(digitalRead(X_RIGHT_LIMIT));
      Serial.print("YF:");
      Serial.print(digitalRead(Y_FORWARD_LIMIT));
      Serial.print("YB:");
      Serial.println(digitalRead(Y_BACKWARD_LIMIT));
    }
    if(digitalRead(X_LEFT_LIMIT)) { // set movement vectors toward home unless that vector is at limit
      xVector = -99; // one off to prevent adding to zero below
    } else {
      xVector = 0;
    }
    
    if(digitalRead(Y_FORWARD_LIMIT)) {
      yVector = 100;
    } else {
      yVector = 0;
    }

    if((xVector + yVector) == 0) { // we're at home
      if (DEBUG) Serial.println("Case: AtHome");
      // run claw opening, 
      if (winchMode == WINCH_MODE_HOMING ) { // ok, so if we came here first time, we need to shift winch mode
         winchMode = runWinch(WinchMotor, ClawMotor, WINCH_CLAW_OPEN_AT_HOME);
      } else if (winchMode == WINCH_MODE_STANDBY) { // ok, if we are here then winch has opened and we're done with our whole game cycle
        overallMode = MODE_WAIT_FOR_GAME;
      } else {
         winchMode = runWinch(WinchMotor, ClawMotor, winchMode); // else, we've already been here so keep running till winch mode adavances itself.
      }

           

      
    } else { // we're not at home so keep moving, maintain the mode
      if (DEBUG) Serial.println("Case: ReturnHome");
      moveXY(xVector, yVector, stepperX, stepperY);
    }

    
    break;
 }

 //if(digitalRead(WINCH_LIMIT) == LOW)  Serial.println("WinchLim");
 
 //if(digitalRead(CLAW_BUTTON) == LOW) Serial.println("ClawDrop");

 
 if(DEBUG) delay(500);
}

void moveXY(int xVector, int yVector, AccelStepper xStepper, AccelStepper yStepper) {
 if(abs(xVector) >= CENTER_TRIM) { // if we're not in the center of the joystick 
   
   // check Y limit and if the vector is in the direction of the limit
   //     set vector to middle if at the particular limit
   //     note, pins are pull up and will go LOW when limit is detected
 
    if ((digitalRead(X_LEFT_LIMIT) == LOW) && (xVector < 0)) {
      xVector= 0;
      if(DEBUG) Serial.println("at X left limit");
    }
   
    if ((digitalRead(X_RIGHT_LIMIT) == LOW) && (xVector >= 0)) {
      xVector= 0;
      if(DEBUG) Serial.println("at X right limit");
    }
    
   stepperX.moveTo(stepperX.currentPosition() + xVector); // move farther in magnitude of vector
   if(DEBUG) printInfo('x', xJoystickVal, xVector, true); 
  } else { // we're in the center of the joystick
   stepperX.moveTo(stepperX.currentPosition()); // move to the current position (i.e. STOP moving)
    if(DEBUG) printInfo('x', xJoystickVal, xVector, false); 
 } 

 if(abs(yVector) >= CENTER_TRIM) { // if we're not in the center of the joystick
  
   // check Y limit and if the vector is in the direction of the limit
   //     set vector to middle if at the particular limit
   //     note, pins are pull up and will go LOW when limit is detected
   
    if ((digitalRead(Y_FORWARD_LIMIT) == LOW) && (yVector >= 0)) {
      yVector= 0;
      if(DEBUG) Serial.println("at Y forward limit");
    }
   
    if ((digitalRead(Y_BACKWARD_LIMIT) == LOW) && (yVector < 0)) {
      yVector= 0;
      if(DEBUG) Serial.println("at Y backward limit");
    }
    
    stepperY.moveTo(stepperY.currentPosition() + yVector); // move farther in magnitude of vector
    
    if(DEBUG) printInfo('y', yJoystickVal, yVector, true); 
  
  } else { // we're in the center of the joystick
   stepperY.moveTo(stepperY.currentPosition()); // move to the current position (i.e. STOP moving)
    if(DEBUG) printInfo('y', xJoystickVal, yVector, false); 
 } 
 stepperX.run();
 stepperY.run(); 
}

void printInfo(char axis, int joystick, int vector, boolean moved) {
   Serial.print("Ax:");
   Serial.print(axis);
   Serial.print("Val:");
   Serial.print(joystick);
   Serial.print("Vect:");
   Serial.print(vector);
   Serial.print("Mov:");
   Serial.println(moved);
}

int setNewMessage(int newMessage) {
  alpha4.clear();
  if (DEBUG) Serial.println("resetting message");
  alpha4.writeDisplay();
  scrollDisplay(newMessage, 0);
  return 1;
}

int scrollDisplay(int currMessage, int currPosition) {
  if ((millis() - lastStep) > SCROLL_DELAY) {
    lastStep=millis();
    if (currPosition > messages[currMessage].length()-1) {
      nextChar = messages[currMessage][0]; 
      currPosition = 1;
      Serial.println(" ");
    } else {
       nextChar = messages[currMessage][currPosition];
       currPosition++;
    }
    
    //scroll display
    displaybuffer[0] = displaybuffer[1];
    displaybuffer[1] = displaybuffer[2];
    displaybuffer[2] = displaybuffer[3];
    displaybuffer[3] = nextChar;
    if (DEBUG) Serial.print(nextChar);
    
    //set the digits from the new buffer
    alpha4.writeDigitAscii(0, displaybuffer[0]);
    alpha4.writeDigitAscii(1, displaybuffer[1]);
    alpha4.writeDigitAscii(2, displaybuffer[2]);
    alpha4.writeDigitAscii(3, displaybuffer[3]);
    
    //write it out
    alpha4.writeDisplay();
  }
  return currPosition;
}

void gameTimerDisplay(int displayNumber) {
  
    alpha4.writeDigitAscii(0, ' ');
    alpha4.writeDigitAscii(1, ' ');
    alpha4.writeDigitAscii(2, (displayNumber/10)+ 48);
    alpha4.writeDigitAscii(3, (displayNumber%10)+ 48);
    
    //write it out
    alpha4.writeDisplay();
  
}

int runWinch(Adafruit_DCMotor *theWinchMotor, Adafruit_DCMotor *theClawMotor,int currentWinchMode) {
  switch(currentWinchMode) {
    case WINCH_MODE_STANDBY: // holding pattern between cycles, if this method is called, we should move straight to RESET
       // because this is never called from the loop due to the if statement, we should never be here.
       
       currentWinchMode = WINCH_MODE_RESETTING; //advance the mode
    break;
 
    case WINCH_MODE_RESETTING:
      if(digitalRead(WINCH_LIMIT) == LOW) { //if the winch is at top, limit switch closed, then set mode to lowering, else raise the motor.
        
        theWinchMotor->run(RELEASE); //stop the motor
        currentWinchMode = WINCH_CLAW_RESETTING; //advance the mode
      } else {
        
        theWinchMotor->run(BACKWARD); // raise the winch
        WinchMotor->setSpeed(WINCH_SPEED);  // set slowish speed.
        // maintain the mode.
      }
    break;
    
    case WINCH_CLAW_RESETTING:
      instantReading = readClaw(CLAW_POSITION);
      if(DEBUG) Serial.println(instantReading);
      if(instantReading >= CLAW_TOTAL_OPEN) {
        ClawMotor->run(RELEASE); // if we are at full open, release motor, advaance the mode
        currentWinchMode = WINCH_MODE_LOWERING;
        if(DEBUG) Serial.println("FullOpenReset");
      } else { //we're not open, so lets getr opened up.
        if(DEBUG) Serial.println("OpenClawReset");
        ClawMotor->run(FORWARD); // maintain current winch mode.
        ClawMotor->setSpeed(CLAW_SPEED);
      }
    break;
    
    case WINCH_MODE_LOWERING:
      if(winchLowerTimer == 0) { // if zero, then we're just starting, so start winch down and set the timer.
        theWinchMotor->run(FORWARD); // lower the winch, maintain the mode.
        WinchMotor->setSpeed(WINCH_SPEED);
        winchLowerTimer = millis();
      } else if ((millis()-winchLowerTimer) > WINCH_LOWER_TIME) { // ok, now we've exceeded the timer, stop winch, set timer zero and advance the mode.
        theWinchMotor->run(RELEASE); // lower the winch
        winchLowerTimer = 0;
        currentWinchMode = WINCH_CLAW_CLOSING; 
        
      } // if neither, continue running, maintain the mode.  
    break;

    case WINCH_CLAW_CLOSING:
      
      instantReading = readClaw(CLAW_POSITION);
      if(DEBUG) Serial.println(instantReading);
      if((instantReading <= CLAW_TOTAL_CLOSE) || rateTooSlow()) {  // if fully closed OR we're closing too slowly, then stop
        ClawMotor->run(RELEASE);
        currentWinchMode = WINCH_MODE_RAISING;
        if(DEBUG) Serial.println("FullyClosed");
      } else {
        ClawMotor->run(BACKWARD);
        ClawMotor->setSpeed(CLAW_SPEED);
        if(DEBUG) Serial.println("Closing");
      }
    break; 

    case WINCH_MODE_RAISING:
      if(digitalRead(WINCH_LIMIT) == LOW) { //if the winch is at top, limit switch closed, then set mode to lowering, else raise the motor.
        theWinchMotor->run(RELEASE); //stop the motor
        currentWinchMode = WINCH_MODE_HOMING; //advance the mode
      } else {
        theWinchMotor->run(BACKWARD); // raise the winch
        WinchMotor->setSpeed(WINCH_SPEED/2);  // set speed. half speed to slow it down
        // maintain the mode.
      }
    break;

    case WINCH_MODE_HOMING:
      
      // basically waiting for program to advance the mode itself
    break;
    
    case WINCH_CLAW_OPEN_AT_HOME:      
      
      instantReading = readClaw(CLAW_POSITION);
      if(DEBUG) Serial.println(instantReading);
      
      if((instantReading >= CLAW_TOTAL_OPEN)) {  // if fully open then stop, else run backward to open
        ClawMotor->run(RELEASE);
        currentWinchMode = WINCH_MODE_STANDBY; // advance the mode.
        if(DEBUG) Serial.println("OpenClawHome");
      } else {
        ClawMotor->run(FORWARD);
        ClawMotor->setSpeed(CLAW_SPEED); //maintain the mode.
        if(DEBUG) Serial.println("OpeningHome");
      }
    break;
  }
  return currentWinchMode;
}

boolean rateTooSlow() {
  
  currentClawReading = analogRead(CLAW_POSITION);
  if(DEBUG) Serial.println(currentClawReading);
  
  if (currentClawReading != lastClawReading) {
    
    lastClawChange = millis(); // reset timer because there is movement
    lastClawReading = currentClawReading; // and set the new reading to the previous reading for next cycle
    
  } else { // ok no movement perceived, so has it been too long?
    
    if((millis() - lastClawChange) > CLAW_CHANGE_TIME_LIM) { // has the difference between now and last change exceeeded time lim

      if (DEBUG) Serial.println("RateTooSlow!");
      return 1; // if yes, then return with a TRUE too slow!

    }
        
    
  }
  return 0; // ok, either we detected movement or we didn't and it hasn't been too long yet.
}

int readClaw(int inputPin) {
  int firstRead = analogRead(inputPin);
  delay(1);
  int secondRead = analogRead(inputPin);
  return (firstRead+secondRead)/2;
}
