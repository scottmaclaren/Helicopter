#include <elapsedMillis.h>
elapsedMillis timeElapsed;
#define encoder0PinA 2
#define encoder0PinB 3
#define motor_pin 10
#define AngleScaleFactor 0.9
#define vert_motor 6

int vert_duty = 255;

volatile int encoder0Pos = 0;
volatile int dutyCycle;

double desired_angle = 180, actual_angle = 0, last_angle = 0;
double error = 0, Integral = 0, IntThresh = 90;
double Motor;   // Value used to determine duty cycle


void setup() {

  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
// encoder pin on interrupt 0 (pin 2)

  pinMode(motor_pin, OUTPUT);
  pinMode(vert_motor, OUTPUT);


  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)

  attachInterrupt(1, doEncoderB, CHANGE);  
  Serial.begin (9600);
  encoder0Pos = 0;
  dutyCycle = RotationPID();
  timeElapsed = 0;
  
}

void loop(){ 

  dutyCycle = RotationPID();
  //Serial.print(dutyCycle);
  //Serial.print(" ");
  
  //delay(100); //added this in for a python test
  //Serial.print(" ");
  //Serial.println(Integral);
  analogWrite(motor_pin, dutyCycle);
  analogWrite(vert_motor, vert_duty);

  if (timeElapsed >100) {
  Serial.println(actual_angle);
  timeElapsed = 0;
  }
  }

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
  double Kp = 7.3, Ki = 0.1, Kd = 22;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  actual_angle = encoder0Pos*AngleScaleFactor;
  error = desired_angle - actual_angle;

  if ((abs(error) <= IntThresh) and (abs(error) >= 3)){ // prevent integral 'windup'
    Integral = Integral + error; // accumulate the error integral
  }
  /*else {
    Integral=0; // zero it if out of bounds
  }*/
 
  P = error*Kp; // calc proportional term
  I = Integral*Ki; // integral term
  D = (last_angle - actual_angle)*Kd; // derivative term
  Motor = P + I + D; // Total drive = P+I+D
  Motor = Motor*PIDScaleFactor;

// Calculate the appropriate duty cycle and direction.
// Values 128 and above are forward direction, 127 and below is backward.
// Minimum value (max backward speed) is 0; Maximum value (max forward speed) is 255

 if (Motor >= 0){ // Check which direction to go.
  Motor = 128 + Motor;
  if (Motor  > 190){
    Motor = 190;
  }
 }
 else { // depending on the sign of Error
  Motor = 127 + Motor;
  if (Motor < 100) {
    Motor = 100;
  }
 }

  last_angle = actual_angle;
 
  return int(Motor);

}
