# line-follower-robot
Developed an autonomous Line Follower Robot capable of navigating predefined paths using IR sensors and PID control. The robot integrates an Arduino microcontroller, L298N motor driver, and DC motors for precise path-following. Designed and implemented robust hardware and software systems, optimizing sensor accuracy and motor performance. 
// Motor Driver Pins
#define ENA 9      // PWM for Motor A
#define ENB 10     // PWM for Motor B
#define IN1 6      // Motor A direction
#define IN2 7      
#define IN3 8      // Motor B direction
#define IN4 9      

// Sensor Configuration
#define NUM_SENSORS 5
int sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4 };
int sensorValues[NUM_SENSORS];

// PID and PDI parameters (Fine-tuned for high accuracy)
float Kp = 7.2, Ki = 0.01, Kd = 3.8; // PID tuning
float Kp_pdi = 2.8, Ki_pdi = 0.1, Kd_pdi = 2.2; // PDI tuning

int baseSpeed = 190; // Slightly increased base speed for stability
int maxSpeed = 255;  
int minSpeed = 60;    

int lastError = 0, integral = 0;
int lastErrorPDI = 0, integralPDI = 0;

void setup() {
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop() {
    int position = readSensors();  
    int error = position - 2;  // Assuming center position is 2

    // Apply noise filtering
    if (abs(error) < 0.5) error = 0;  

    // PID Calculation
    int P = error;
    integral += error;
    int I = integral;
    int D = error - lastError;
    int correctionPID = (Kp * P) + (Ki * I) + (Kd * D);
    lastError = error;

    // PDI Calculation (Smooth response)
    int P_pdi = error;
    integralPDI += error;
    int I_pdi = integralPDI;
    int D_pdi = error - lastErrorPDI;
    int correctionPDI = (Kp_pdi * P_pdi) + (Ki_pdi * I_pdi) + (Kd_pdi * D_pdi);
    lastErrorPDI = error;

    // PWM Control: Adjust speeds using both PID and PDI corrections
    int leftSpeed = baseSpeed + correctionPID - correctionPDI;
    int rightSpeed = baseSpeed - correctionPID + correctionPDI;

    // Ensure speeds remain within valid range
    leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

    moveMotors(leftSpeed, rightSpeed);
}

// **Improved Sensor Reading - Weighted Average Calculation**
int readSensors() {
    int position = 0, totalValue = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
        position += sensorValues[i] * i;   // Weighting each sensor's value
        totalValue += sensorValues[i];
    }
    return (totalValue > 0) ? (position / totalValue) : 2;  
}

// **Motor Control with Smooth PWM**
void moveMotors(int left, int right) {
    analogWrite(ENA, left);  // PWM Speed Control
    analogWrite(ENB, right);
    
    digitalWrite(IN1, left > 0 ? HIGH : LOW);
    digitalWrite(IN2, left > 0 ? LOW : HIGH);
    digitalWrite(IN3, right > 0 ? HIGH : LOW);
    digitalWrite(IN4, right > 0 ? LOW : HIGH);
}
