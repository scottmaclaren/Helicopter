int IRpin = A0;               // IR photodiode on analog pin A0
int IRemitter = 2;            // IR emitter LED on digital pin 2
int ambientIR;                // variable to store the IR coming from the ambient
int obstacleIR;               // variable to store the IR coming from the object
int value[10];                // variable to store the IR values
int distance;                 // variable that will tell if there is an obstacle or not
int vertical_motor = 6;
int dutyCycle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);         // initializing Serial monitor
  pinMode(IRemitter,OUTPUT);  // IR emitter LED on digital pin 2
  digitalWrite(IRemitter,LOW);// setup IR LED as off
  pinMode(11,OUTPUT);         // buzzer in digital pin 11
  pinMode(vertical_motor, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  distance = readIR(5);       // calling the function that will read the distance and passing the "accuracy" to it
  Serial.println(distance); 
  Serial.print("\t");   // writing the read value on Serial monitor

  dutyCycle = VerticalPID(distance);
  Serial.print(dutyCycle);
  Serial.print("\n");

  analogWrite(vertical_motor, dutyCycle);
  
}

int readIR(int times){
  for(int x=0;x<times;x++){     
    digitalWrite(IRemitter,LOW);           // turning the IR LEDs off to read the IR coming from the ambient
    delay(1);                                             // minimum delay necessary to read values
    ambientIR = analogRead(IRpin);  // storing IR coming from the ambient
    digitalWrite(IRemitter,HIGH);          // turning the IR LEDs on to read the IR coming from the obstacle
    delay(1);                                             // minimum delay necessary to read values
    obstacleIR = analogRead(IRpin);  // storing IR coming from the obstacle
    value[x] = ambientIR-obstacleIR;   // calculating changes in IR values and storing it for future average
  }
 
  for(int x=0;x<times;x++){        // calculating the average based on the "accuracy"
    distance+=value[x];
  }
  return(distance/times);            // return the final value
}

double VerticalPID(int distance) {

  double desired_distance = 0, actual_distance = distance, last_distance = distance;
  //double AngleScaleFactor = 0.75; 
  double PIDScaleFactor = 0.15;
  double error = 0, Integral = 0, IntThresh = 15;
  double Kp = 5, Ki = 1, Kd = 0;       // PID constants
  double P = 0, I = 0, D = 0;         //  Proportional, Integral, and Derivative terms to be summed
  double Motor = 0;   // Value used to determine duty cycle

  //actual_angle = encoder0Pos*AngleScaleFactor; // **** TEST THIS ****
  error = actual_distance - desired_distance;

  if (abs(error) < IntThresh){ // prevent integral 'windup'
    Integral = Integral + error; // accumulate the error integral
  }
  else {
    Integral=0; // zero it if out of bounds
  }
 
  P = error*Kp; // calc proportional term
  I = Integral*Ki; // integral term
  D = (last_distance - actual_distance)*Kd; // derivative term
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

  last_distance = actual_distance;
 
  return int(Motor);

}

