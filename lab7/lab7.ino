/*
 * CSCI 1063U - Elegoo Smart Car V4.0
 *
 * Starter Code for motor, pin, and sensor setup
 * Provided to students for use and understanding
 * 
 */

#include <Wire.h>
#include <FastLED.h>
#include <Servo.h>

// ====== PIN CONSTANT VALUES ======

#define NUM_LEDS 2            // Number of LEDs on your board
#define PIN_RBGLED 4          // LED Pin
#define PWR_R 5               // Right Motor Power
#define PWR_L 6               // Left Motor Power
#define MTR_L 7               // Left Motor Control
#define MTR_R 8               // Right Motor Control
#define SERVO 10              // Servo Motor
#define MTR_ENABLE 3          // Motor Enable Pin
#define US_OUT 13             // Ultrasonic Sensor Input
#define US_IN 12              // Ultrasonic Sensor Output
#define LINE_L A2             // Left Line Tracker
#define LINE_C A1             // Center Line Tracker
#define LINE_R A0             // Right Line Tracker
#define BUTTON 2              // Push Button
#define GYRO 0x68             // Gyro Sensor Address

#define NORMAL_SPEED 50
#define TURN_SPEED 100
#define LINE_THRESHOLD 300

#define BRIGHTNESS 100

#define DISTANCE_LOOKBACK 5
#define MIN_DISTANCE 5
#define MAX_DISTANCE 20

#define GYRO_LOOKBACK 5
#define GYRO_EPISLION 2

#define MAX_SPEED 150
#define MIN_SPEED 25

int16_t gyroZ;                // Raw gyro Z-axis reading
float gyroZOffset = 0;        // Calibration offset
float currentAngle = 0;       // Current angle in degrees
unsigned long lastTime = 0;   // Last read time
Servo scanServo;              // Servo

int state = 0;
int delta_turn = 0;
int avg_distance = 0; 
int avg_angle = 0;
int timer = 50;

long randNumber;

int distances[DISTANCE_LOOKBACK] = {MAX_DISTANCE};
uint8_t d_i = 0;

int angles[GYRO_LOOKBACK] = {0};
uint8_t g_i = 0;
// --------------------------------------------


void setServoAngle(int angle, int rest) {
  static int lastAngle = -1;
  angle = constrain(angle, 0, 180);

  rest = (rest < 0) ? 15 : rest;

  if (angle != lastAngle) {
    scanServo.write(angle);
    delay(rest);  // Allow servo to settle
    lastAngle = angle;
  }
}

void centerServo(int d) {
  setServoAngle(90, d);
}

bool setupGyro() {
  Wire.begin();
  Wire.beginTransmission(GYRO);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Configure gyro sensitivity (±250 deg/s)
  Wire.beginTransmission(GYRO);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // ±250 deg/s
  Wire.endTransmission();
  
  lastTime = millis();
  return true;
}

// Calibrate gyro (robot must be stationary!)
void calibrateGyro() {
  delay(500);
  
  long sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(GYRO);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(GYRO, 2, true);
    
    int16_t gz = Wire.read() << 8 | Wire.read();
    sum += gz;
    delay(10);
  }
  
  gyroZOffset = sum / samples;
  currentAngle = 0;
}

int16_t readGyroZ() {
  Wire.beginTransmission(GYRO);
  Wire.write(0x47);  // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO, 2, true);
  
  int16_t gz = Wire.read() << 8 | Wire.read();
  return gz;
}

// MUST be called frequently (e.g., every loop iteration)
// Angle accuracy degrades if this is not called often
void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // Time in seconds
  lastTime = now;
  
  // Read gyro
  gyroZ = readGyroZ();
  
  // Convert to degrees per second (sensitivity = 131 for ±250 deg/s)
  // INVERTED THE SIGN HERE to fix direction!
  float gyroRate = -((gyroZ - gyroZOffset) / 131.0);
  
  // Integrate to get angle
  currentAngle += gyroRate * dt;
  
  // Keep angle in range -180 to +180
  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;
}

void resetAngle() {
  currentAngle = 0;
}

float getAngle() {
  return currentAngle;
}

int getDistance() {
  int validReading = 0;
  int attempts = 0;
  
  while (validReading == 0 && attempts < 3) {
    if (attempts > 0) delay(60);  // Only delay on retries
    
    digitalWrite(US_OUT, LOW);
    delayMicroseconds(2);
    digitalWrite(US_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_OUT, LOW);
    
    long duration = pulseIn(US_IN, HIGH, 30000);
    int distance = duration * 0.034 / 2;
    
    if (duration > 0 && distance <= 200) {
      validReading = distance;
    }
    
    attempts++;
  }
  
  return validReading;
}

void setup() {
  // setup LED
  FastLED.setBrightness(50); // 0-255
  
  // Motor pins
  pinMode(PWR_R, OUTPUT);
  pinMode(PWR_L, OUTPUT);
  pinMode(MTR_L, OUTPUT);
  pinMode(MTR_R, OUTPUT);
  pinMode(MTR_ENABLE, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(US_OUT, OUTPUT);
  pinMode(US_IN, INPUT);

  // Line tracking sensor pins
  pinMode(LINE_L, INPUT);
  pinMode(LINE_C, INPUT);
  pinMode(LINE_R, INPUT);
  
  // Button pin
  pinMode(BUTTON, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(MTR_ENABLE, HIGH);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Servo motor
  scanServo.attach(SERVO);
  centerServo(-1);   // Center position
  
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {
    Serial.println("Button not pressed");
  }

  delay(500);

  // Initialize Gyro - hard stop if failed
  if (!setupGyro()) {
    Serial.println("Failed setting up gyro...");
    while (true);  // Hard stop
  }

  calibrateGyro();

  // ----------------------------------------------
  randomSeed(analogRead(0));

  print("MAIN", "setup done", 0);
}




void print(String system, String msg, int d)
{
  Serial.print("[" + system + "] " + msg + ": ");
  Serial.println(d);
}

void update_array(int* array, int length, uint8_t i, int value)
{
  array[i % length] = value;
}

int get_array_average(int* array, int length)
{
  int total = 0;

  for (int i = 0; i < length; i++)
  {
    total += array[i];
  }
  return total / length;
}

void reset_array(int* array, int length, int val)
{
  for (int i = 0; i < length; i++)
  {
    array[i] = val;
  }
}

// --------------------------------------------------------------------
bool wall_is_ahead()
{
  return avg_distance < MIN_DISTANCE;
}

void move_motors(int lspeed, int rspeed)
{
  if (lspeed >= 0) {
    digitalWrite(MTR_L, HIGH);
  }
  else {
    digitalWrite(MTR_L, LOW);
  }

  if (rspeed >= 0) {
    digitalWrite(MTR_R, HIGH);
  }
  else {
    digitalWrite(MTR_R, LOW);
  }

  analogWrite(PWR_L, constrain(abs(lspeed), 0, 255));
  analogWrite(PWR_R, constrain(abs(rspeed), 0, 255));
}

bool white_is_under(int val)
{
  return val < LINE_THRESHOLD;
}


// --------------------------------------------------------------------
void driver()
{
  int left = analogRead(LINE_L);
  int center = analogRead(LINE_C);
  int right = analogRead(LINE_R);

  bool white_center = white_is_under(center);
  bool white_left = white_is_under(left);
  bool white_right = white_is_under(right);

  // move_motors(NORMAL_SPEED, NORMAL_SPEED);


  if (white_left && white_right && white_center){
    reverse();
    turn_90();
  }

  if (white_left){
    print("TURN", "turning left", 0);
    move_motors(NORMAL_SPEED + 50, NORMAL_SPEED - 50);
  }

  else if (white_right){
    print("TURN", "turning right", 0);
    move_motors(NORMAL_SPEED - 50, NORMAL_SPEED + 50);
  }

  else {
    move_motors(NORMAL_SPEED + 5, NORMAL_SPEED);
  }

  if (wall_is_ahead())
  {
    delta_turn = -90;
  }

}


void reverse(){
  move_motors(-75 - 5, -75);
  delay(100);
  move_motors(NORMAL_SPEED + 5, NORMAL_SPEED);
}


void turn_90(){
  move_motors(-100 + 5, 100);
  delay(500);
  move_motors(NORMAL_SPEED + 5, NORMAL_SPEED);
}

void turn()
{
  if (abs(delta_turn) < GYRO_EPISLION)
  {
    resetAngle();
    state = 0;
  }

  print("main", "delta_turn =", delta_turn);
  print("main", "avg_angle =", getAngle());
  delta_turn -= getAngle();
  resetAngle();

  if (delta_turn < 0)
  {
    // print("TURN", "turning left", 0);
    move_motors(-NORMAL_SPEED, NORMAL_SPEED);
  }
  else if (delta_turn > 0)
  {
    // print("TURN", "turning right", 0);
    move_motors(NORMAL_SPEED, -NORMAL_SPEED);
  }
}



// STATES
// -1 TESTING PURPOSES (I'm sick and tired of the motor noises)
// 0 STRAIGHT
// 1 TURN

void loop()
{
  updateGyroAngle();

  
  // Getting averages
  update_array(distances, DISTANCE_LOOKBACK, d_i++, getDistance());
  avg_distance = get_array_average(distances, DISTANCE_LOOKBACK);

  if (avg_distance < MIN_DISTANCE)
  {
    move_motors(0, 0);
    state = -1;
  }
  switch (state)
  {
    case -1:
      break;

    case 0:
      driver();
      break;
    
    case 1:
      // turn();
      break;

    default:
      break;
  }
}
