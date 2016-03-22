#define encoder0PinA 2
#define encoder0PinB 3
#define motor_pin 10
#define AngleScaleFactor 0.9

char state = 0;
boolean target_reached = false;
boolean start = false;

volatile int encoder0Pos = 0;
volatile int dutyCycle = 0;

double desired_angle = 180, actual_angle = 0, last_angle = 0;
double error = 0, Integral = 0, IntThresh = 30;
double Motor = 0;   // Value used to determine duty cycle


void setup() {

  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
// encoder pin on interrupt 0 (pin 2)

  pinMode(motor_pin, OUTPUT);

  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)

  attachInterrupt(1, doEncoderB, CHANGE);  
  Serial.begin (9600);
}

void loop(){ 

  if (state == 0){
    //TURN OFF BOTH MOTORS
    
    if(start == 1){ //read in start from keyboard or something to that extent
      state = 1;
      start == false;
    }
  }

  else if(state == 1){
    //Run Vertical PID with target = XX
    //Keep rotational motor off

    if(target_reached == true){ //do multiple 
      state = 2;
      target_reached == false;
    }
  }
  else if(state == 2){
    //Run vertical motor at steady duty cycle of _____
    //Run Horizontal PID
    
    if(target_reached == true){
      state = 3;
      target_reached = false;
    }
  }
  else if(state == 3){
    //Lower vertical motor: duty cycle = _____
    // Turn of rotational motor

    if(target_reached == true){
      state = 0;
      target_reached = false;
    }
  }

  Serial.print(dutyCycle);
  Serial.print(" ");
  Serial.println(encoder0Pos);
  analogWrite(motor_pin, dutyCycle);
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

  dutyCycle = RotationPID(encoder0Pos);
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

  dutyCycle = RotationPID(encoder0Pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to determine and return the PWM duty cycle for the rotational motor using a PID controller
///////////////////////////////////////////////////////////////////////////////////////////////////////

double RotationPID(int encoder0pos) {
  
  double PIDScaleFactor = 0.15;
  double Kp = 5, Ki = 0, Kd = 0;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  actual_angle = encoder0Pos*AngleScaleFactor;
  error = desired_angle - actual_angle;

  if (abs(error) < IntThresh){ // prevent integral 'windup'
    Integral = Integral + error; // accumulate the error integral
  }
  else {
    Integral=0; // zero it if out of bounds
  }
 
  P = error*Kp; // calc proportional term
  I = Integral*Ki; // integral term
  D = (last_angle - actual_angle)*Kd; // derivative term
  Motor = P + I + D; // Total drive = P+I+D
  Motor = Motor*PIDScaleFactor;

// Calculate the appropriate duty cycle and direction.
// Values 128 and above are forward direction, 127 and below is backward.
// Minimum value (max backward speed) is 0; Maximum value (max forward speed) is 255

 if (Motor > 0){ // Check which direction to go.
  Motor = 128 + Motor;
  if (Motor  > 255){
    Motor = 255;
  }
 }
 else { // depending on the sign of Error
  Motor = 127 + Motor;
  if (Motor < 0) {
    Motor = 0;
  }
 }

  last_angle = actual_angle;
 
  return int(Motor);

}
