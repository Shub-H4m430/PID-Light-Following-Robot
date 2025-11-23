// --- PIN DEFINITIONS ---
const int pwm1 = 10; // Right Motor Speed (ENA)
const int pwm2 = 11; // Left Motor Speed (ENB)
const int in1 = 4;   // Right Motor Direction
const int in2 = 7;   
const int in3 = 9;   // Left Motor Direction
const int in4 = 8;   

// Analog Sensor Pins
const int leftLdr = A0;
const int centeredLdr = A1;
const int rightLdr = A2;

// --- TUNING PARAMETERS (PID) ---
float Kp = 0.15;   // Proportional Gain (Reacts to current error)
float Ki = 0.0;    // Integral Gain (Accumulates past error)
float Kd = 1.0;    // Derivative Gain (Predicts future error/Damping)

// --- SYSTEM VARIABLES ---
int baseSpeed_right = 90; // Base PWM for Right Motor
int baseSpeed_left = 110;  // Base PWM for Left Motor (compensated for hardware drag)

int lastError = 0;
int integral = 0;

// Threshold for "Darkness" (Safety Stop)
const int darkThreshold = 800; 
const int brightThreshold = 500;

void setup() {
  // 1. Initialize Serial Communication (Required for Task)
  Serial.begin(9600);
  Serial.println("--- PID Light Follower Initialized ---");
  Serial.println("Left | Center | Right | Error | R_Speed | L_Speed");

  // 2. Initialize Motor Pins
  pinMode(pwm1, OUTPUT); pinMode(pwm2, OUTPUT);
  pinMode(in1, OUTPUT);  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);  pinMode(in4, OUTPUT);

  // 3. Initialize Sensors
  pinMode(rightLdr, INPUT);
  pinMode(centeredLdr, INPUT);
  pinMode(leftLdr, INPUT);
}

void loop() {
  // --- STEP 1: ACQUIRE DATA ---
  int rightVal = analogRead(rightLdr);
  int leftVal = analogRead(leftLdr);
  int centerVal = analogRead(centeredLdr);

  // --- STEP 2: THRESHOLD LOGIC (Safety & Reverse) ---
  
  // Scenario A: Too Bright (Light is very close) -> Reverse
  if(centerVal < brightThreshold){
    // Actuator Action: Reverse Motors
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
    analogWrite(pwm1, baseSpeed_right);
    analogWrite(pwm2, baseSpeed_left);
    Serial.println("Status: REVERSING (Too Bright)");
  }
  
  // Scenario B: Standard PID Following
  else {
    // Set Direction to FORWARD
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);

    // Safety Stop: If environment is too dark
    if (rightVal > darkThreshold && leftVal > darkThreshold) {
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
      Serial.println("Status: STOPPED (Too Dark)");
      return; 
    }

    // --- STEP 3: PID ALGORITHM ---
    // Calculate Error (Positive = Light on Left)
    // Note: Sensors are Pull-Up (Low Val = Bright), so we subtract Left from Right
    int error = rightVal - leftVal;

    int P = error;
    integral += error; 
    int D = error - lastError;

    int PID_value = (Kp * P) + (Ki * integral) + (Kd * D);

    // --- STEP 4: ACTUATOR CONTROL (Motor Mixing) ---
    int rightMotorSpeed = baseSpeed_right + PID_value;
    int leftMotorSpeed = baseSpeed_left - PID_value;

    // Constrain PWM to valid range (0-255)
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

    analogWrite(pwm1, rightMotorSpeed);
    analogWrite(pwm2, leftMotorSpeed);

    lastError = error;

    // --- STEP 5: SERIAL LOGGING (Required) ---
    Serial.print(leftVal); Serial.print(" \t ");
    Serial.print(centerVal); Serial.print(" \t ");
    Serial.print(rightVal); Serial.print(" \t ");
    Serial.print(error); Serial.print(" \t ");
    Serial.print(rightMotorSpeed); Serial.print(" \t ");
    Serial.println(leftMotorSpeed);
  }
  
  delay(10); // Stability delay
}