#define AnglePerVal 0.29
#define Target_Angle 45

int vertical_motor = 6;
int dutyCycle;

int potPin = A0;
int starting_val;
int angle = 0;
double last_val;
double target_val;
double current_val;

double error = 0, Integral = 0, IntThresh = 30;
double Motor = 0;   // Value used to determine duty cycle

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);      
  pinMode(vertical_motor, OUTPUT);
  starting_val = analogRead(potPin);
  target_val = starting_val + Target_Angle/AnglePerVal; 
}

void loop() {
  // put your main code here, to run repeatedly:

   current_val = analogRead(potPin);
   angle = current_val - starting_val;
   angle = angle*AnglePerVal;
   Serial.print(angle);
   Serial.print(" ");
  
  dutyCycle = VerticalPID(current_val);
  Serial.println(dutyCycle);

  analogWrite(vertical_motor, dutyCycle);
  
}

//int readIR(int times){ legacy code- I don't think we need this anymore? SM
// 
//}

double VerticalPID(double current_val) {
    
  double PIDScaleFactor = 0.15;
  double Kp = 5, Ki = 0, Kd = 0;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed

  //actual_angle = encoder0Pos*AngleScaleFactor; // **** TEST THIS ****
  error = target_val - current_val;

  if (abs(error) < IntThresh){ // prevent integral 'windup'
    Integral = Integral + error; // accumulate the error integral
  }
  else {
    Integral=0; // zero it if out of bounds
  }
 
  P = error*Kp; // calc proportional term
  I = Integral*Ki; // integral term
  D = (last_val - current_val)*Kd; // derivative term
  Motor = P + I + D; // Total drive = P+I+D
  Motor = Motor*PIDScaleFactor;

// Calculate the appropriate duty cycle and direction.
// Duty cycles 128 and above are forward direction, 127 and below is backward.
// Minimum duty cycle (max backward speed) is 0; Maximum value (max forward speed) is 255

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

  last_val = current_val;
 
  return int(Motor);

}

