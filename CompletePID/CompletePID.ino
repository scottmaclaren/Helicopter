/* NOTES: 
 *  - Variables with "Yaw" or "yaw" in them are used to deal with the rotation of the mechanism
 *  - Variables with "angle" or "Angle" are used to deal with the angle of rotation of the potentiometer 
 *  - Need a method for determining when a target has been reached within an acceptable range. 
 *    Will try to use some kind of delay and re-check method...
 *  - PID constants have not been updated to the latest ones that were physically tested
 */

 //note this will only work after downloading an additional library if we decide to go forward with it. 
#include <elapsedMillis.h>

// Encoder and Rotational Constants
#define encoder0PinA 2
#define encoder0PinB 3
#define yaw_motor 10
#define YawScaleFactor 0.9

// Potentiometer and Vertical Constants
#define AnglePerVal 0.29
#define Target_Angle 45
#define vertical_motor 6
#define potPin A0

// Tolerable range for rotation and height
#define thresh 3 

// State machine variables and flags
elapsedMillis timeElapsed;
int state = 0;
boolean target_reached = false;
boolean start = false;
int letsgo = 0;

// Rotation PID variables
volatile int encoder0Pos = 0;
int yaw_duty = 0;
double desired_yaw, actual_yaw = 0, last_yaw = 0;
double yaw_error = 0, yaw_Integral = 0, yaw_IntThresh = 90;
double yaw_Motor = 0;   // Value used to determine duty cycle

// Potentiometer PID variables
int starting_pot, current_pot;
int vert_duty;
double target_angle, angle, last_angle;
int pot_angle = 0;
double vert_error = 0, vert_Integral = 0, vert_IntThresh = 30;
double vert_Motor = 0;   // Value used to determine duty cycle


void setup() {

  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
// encoder pin on interrupt 0 (pin 2)

  pinMode(yaw_motor, OUTPUT);
  pinMode(vertical_motor, OUTPUT);

  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)

  attachInterrupt(1, doEncoderB, CHANGE); 

  starting_pot = analogRead(potPin);
  //target_pot = starting_pot + Target_Angle/AnglePerVal;

  Serial.begin (9600);
}

void loop(){ 


  if (state == 0){
    //TURN OFF BOTH MOTORS

    vert_duty = 0;
    yaw_duty = 127;
    analogWrite(yaw_motor, yaw_duty);
    analogWrite(vertical_motor, vert_duty);
    
//we can do a serial read in from the keyboard for the moment
    Serial.println("Please enter a '1' to begin");
  
    while(Serial.available()==0) { // Wait for User to Input Data  
    }
    letsgo=Serial.parseInt(); 
   
   Serial.println("Let the good times roll");
////    //or we can use the pushbutton- just change the pin and we need to add a pull down resistor(then unpushed is 0)
////    //state = digitalRead(4)
////    
    if(letsgo == 1){ //read in start from keyboard or something to that extent. Pushbutton?
      
      state = 1;
      timeElapsed = 0;
      //start == false;
   }
  
  }

  else if(state == 1){
    //Run Vertical PID with target = XX
    //Keep rotational motor off
   
    desired_yaw = 0;
    target_angle = 45;
    
    current_pot = analogRead(potPin);
    angle = starting_pot - current_pot; 
    angle = angle*AnglePerVal;
    
    //vert_duty = VerticalPID();
    vert_duty = 255;
    yaw_duty = RotationPID();
    analogWrite(vertical_motor, vert_duty);
    analogWrite(yaw_motor, yaw_duty);
    //what we can do for the target reached is we can check the time elasped function after X number of seconds and if 
    //we can are in a range of tolerance, we can flag the target_reached as true
    
    if (timeElapsed > 2000){
      state = 2;
      timeElapsed = 0;

      }
      
      
      
    
  }
  else if(state == 2){
    //Run vertical motor at steady duty cycle of _____
    //Run Horizontal PID
    
    desired_yaw = 175;

    yaw_duty = RotationPID();
    analogWrite(yaw_motor, yaw_duty);
    analogWrite(vertical_motor, vert_duty);
    
   
    
    if (timeElapsed > 20000){
     timeElapsed = 0;
      state = 3;
    }
    
  }
  
  
  else if(state == 3){
    //Lower vertical motor: duty cycle = _____
    // Turn of rotational motor
   
    target_angle = 0;
    
    current_pot = analogRead(potPin);
    angle = starting_pot - current_pot; 
    angle = angle*AnglePerVal;
    
  
    vert_duty = VerticalPID();
    yaw_duty = RotationPID();
    analogWrite(vertical_motor, vert_duty);
    analogWrite(yaw_motor, yaw_duty);




    
    if(target_reached == true){
      state = 0;
      
    }
  }
//had to commment this out or it spits
//  Serial.print(yaw_duty);
//  Serial.print(" ");
//Serial.print(encoder0Pos);
//  Serial.print(vert_duty);
//  Serial.println(current_pot);
//  analogWrite(yaw_motor, yaw_duty);
Serial.print(timeElapsed);
Serial.print(" ");
Serial.print(state);
Serial.print(" ");
Serial.print(actual_yaw);
Serial.print(" ");
Serial.println(yaw_duty);

  }


/*
 * Interrupt Sub-routines for the optical encoder. Each time the state of either pin 2 or 3 is
 * changed, an ISR is triggered. The following code determines the direction of rotation based
 * on which pin triggered the interrupt and the current state of the other pin.
 */

void doEncoderA()
{

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);          
  // use for debugging - remember to comment out
 
}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to determine and return the PWM duty cycle for the rotational motor using a PID controller
///////////////////////////////////////////////////////////////////////////////////////////////////////

double RotationPID() {
  
  double PIDScaleFactor = 0.05;
  double Kp = 6.6, Ki = 0.09, Kd = 27;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  actual_yaw = encoder0Pos*YawScaleFactor;
  yaw_error = desired_yaw - actual_yaw;

  if ((abs(yaw_error) <= yaw_IntThresh) and (abs(yaw_error) >= thresh)){ // prevent integral 'windup'
    yaw_Integral = yaw_Integral + yaw_error; // accumulate the error integral
  }
  /*else {
    yaw_Integral=0; // zero it if out of bounds
  }*/
 
  P = yaw_error*Kp; // calc proportional term
  I = yaw_Integral*Ki; // integral term
  D = (last_yaw - actual_yaw)*Kd; // derivative term
  yaw_Motor = P + I + D; // Total drive = P+I+D
  yaw_Motor = yaw_Motor*PIDScaleFactor;

// Calculate the appropriate duty cycle and direction.
// Values 128 and above are forward direction, 127 and below is backward.
// Minimum value (max backward speed) is 0; Maximum value (max forward speed) is 255

 if (yaw_Motor > 0){ // Check which direction to go.
  yaw_Motor = 128 + yaw_Motor;
  if (yaw_Motor  > 190){
    yaw_Motor = 190;
  }
 }
 else { // depending on the sign of Error
  yaw_Motor = 127 + yaw_Motor;
  if (yaw_Motor < 100) {
    yaw_Motor = 100;
  }
 }

  last_yaw = actual_yaw;
 
  return int(yaw_Motor);

}

/*
 *  A PID controller to control the height of the custom-built vertical motor.
 *  Code follows the same algorithm as the rotation PID except that the variable names 
 *  have been changed.
 */

double VerticalPID() {
    
  double PIDScaleFactor = 0.15;
  double Kp = 5, Ki = 0, Kd = 0;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  vert_error = target_angle - angle;

  if ((abs(vert_error) < vert_IntThresh) and (abs(vert_error) > 3*thresh)){ // prevent integral 'windup'
    vert_Integral = vert_Integral + vert_error; // accumulate the error integral
  }
  /*else {
    vert_Integral=0; // zero it if out of bounds
  }*/
 
  P = vert_error*Kp; // calc proportional term
  I = vert_Integral*Ki; // integral term
  D = (last_angle - angle)*Kd; // derivative term
  vert_Motor = P + I + D; // Total drive = P+I+D
  vert_Motor = vert_Motor*PIDScaleFactor;

// Calculate the appropriate duty cycle and direction.
// Duty cycles 128 and above are forward direction, 127 and below is backward.
// Minimum duty cycle (max backward speed) is 0; Maximum value (max forward speed) is 255

 if (vert_Motor > 0){ // Check which direction to go.
  vert_Motor = 128 + vert_Motor;
  if (vert_Motor  > 255){
    vert_Motor = 255;
  }
 }
 else { // depending on the sign of Error
  vert_Motor = 127 + vert_Motor;
  if (vert_Motor < 50) {
    vert_Motor = 50;
  }
 }

  last_angle = angle;
 
  return int(vert_Motor);

}

