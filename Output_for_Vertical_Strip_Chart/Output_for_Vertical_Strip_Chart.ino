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
double vert_error = 0, vert_Integral = 0, vert_IntThresh = 20;
double vert_Motor = 0;   // Value used to determine duty cycle

void setup() {
  // put your setup code here, to run once:

 pinMode(vertical_motor, OUTPUT);


 // attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)

//  attachInterrupt(1, doEncoderB, CHANGE);  
  Serial.begin (9600);
  starting_pot = analogRead(potPin);
  timeElapsed =0;
  
}

void loop() {
  // put your main code here, to run repeatedly:
    desired_yaw = 0;
    target_angle = 31;
    
    current_pot = analogRead(potPin);
    angle = starting_pot - current_pot; 
    angle = angle*AnglePerVal;
    
    vert_duty = VerticalPID();
 
    analogWrite(vertical_motor, vert_duty);
    if (timeElapsed >100) {

      Serial.println(angle);
      timeElapsed =0;
    }

}

double VerticalPID() {
    
  double PIDScaleFactor = 0.13;
  double Kp = 27, Ki = 0.2, Kd = 0;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  vert_error = target_angle - angle;

  if ((abs(vert_error) < vert_IntThresh) and (abs(vert_error) > thresh)){ // prevent integral 'windup'
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
  if (vert_Motor < 110) {
    vert_Motor = 110 ;
  }
 }

  last_angle = angle; 
 
  return int(vert_Motor);

}

