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

// ====== PROGRAM CONSTANTS ======
#define SPEED_NORMAL 50
#define SPEED_TURN 100
#define LINE_THRESHOLD 300

// --------------------------------------------
#define DISTANCE_LOOKBACK 5
#define MIN_DISTANCE 5
#define MAX_DISTANCE 20
#define BRIGHTNESS 100

#define GYRO_LOOKBACK 5
#define GYRO_EPISLION 2

#define SPEED_MAX 150
#define SPEED_MIN 25
// --------------------------------------------

// ====== PROGRAM VARIABLES ======
int16_t gyroZ;                // Raw gyro Z-axis reading
float gyroZOffset = 0;        // Calibration offset
float currentAngle = 0;       // Current angle in degrees
unsigned long lastTime = 0;   // Last read time
CRGB leds[NUM_LEDS];          // Current LED Color values
Servo scanServo;              // Servo


// --------------------------------------------
int state = 0;
long randNumber;

int distances[DISTANCE_LOOKBACK] = {MAX_DISTANCE};
uint8_t d_i = 0;

int angles[GYRO_LOOKBACK] = {0};
uint8_t g_i = 0;
// --------------------------------------------



// ====== LED FUNCTIONS ======
void ledOn(CRGB color) {
  leds[0] = color;
  FastLED.show();
}

void ledOff() {
  leds[0] = CRGB::Black;
  FastLED.show();
}

// ===== SERVO FUNCTIONS =====
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

// ====== GYRO FUNCTIONS ======
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

// ===== ULTRASONIC SENSOR FUNCTIONS =====
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
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
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
    ledOn(CRGB::Red);
    while (true);  // Hard stop
  }

  calibrateGyro();

  // ----------------------------------------------
  randomSeed(analogRead(0));

  print("MAIN", "setup done", 0);
}

// -----------------------------------------------------------------------------------

void print(String system, String msg, int d)
{
  Serial.print("[" + system + "] " + msg + ": ");
  Serial.println(d);
}

void print_distances()
{
  Serial.print("[USS] Distances: ");
  for (int i = 0; i < DISTANCE_LOOKBACK; i++)
  {
    Serial.print(distances[i]);
    Serial.print(" ");
  }
  Serial.println();
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

CRGB get_roaming_colour(int d)
{
  if (d < MIN_DISTANCE) return CRGB(BRIGHTNESS, 0, 0);
  if (d > MAX_DISTANCE) return CRGB(0, 0, BRIGHTNESS);

  int r = -(d - MIN_DISTANCE) * BRIGHTNESS / (MAX_DISTANCE - MIN_DISTANCE) + BRIGHTNESS;
  int g = 0;
  int b = (d - MIN_DISTANCE) * BRIGHTNESS / (MAX_DISTANCE - MIN_DISTANCE);

  return CRGB(r, g, b);
}

CRGB get_following_colour(int d)
{
  if (d < MIN_DISTANCE) return CRGB(0, 0, 0);
  if (d > MAX_DISTANCE) return CRGB(0, BRIGHTNESS, 0);

  int r = 0;
  int g = -(d - MIN_DISTANCE) * BRIGHTNESS / (MAX_DISTANCE - MIN_DISTANCE) + BRIGHTNESS;
  int b = 0;

  return CRGB(r, g, b);
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

void shake_head()
{
  setServoAngle(45, 200);
  setServoAngle(135, 200);

  setServoAngle(45, 200);
  setServoAngle(135, 200);

  setServoAngle(67.5, 200);
  setServoAngle(112, 200);

  centerServo(0);
}

void freak_out()
{
  move_motors(-200, 200);

  shake_head();
  shake_head();
  shake_head();
}

int get_speed(int d)
{
  int result = (SPEED_MAX - SPEED_MIN) / pow((MAX_DISTANCE - MIN_DISTANCE), 2) * pow((d - MIN_DISTANCE), 2) + SPEED_MIN;
  return constrain(result, SPEED_MIN, SPEED_MAX);
}

void roaming()
{
  int avg_distance = 0;
  int avg_angle = 0;
  int dl = 0;
  int dr = 0;

  int center = analogRead(LINE_C);
  bool on_center = center < LINE_THRESHOLD;

  if (on_center)
  {
    state = 3;
    return;
  }

  update_array(distances, DISTANCE_LOOKBACK, d_i++, getDistance());
  update_array(angles, GYRO_LOOKBACK, g_i++, getAngle());

  avg_distance = get_array_average(distances, DISTANCE_LOOKBACK);
  avg_angle = get_array_average(angles, GYRO_LOOKBACK);

  ledOn(get_roaming_colour(avg_distance));

  // If the angle get's off we reset so there isn't runaway addition
  if (abs(avg_angle) >= GYRO_EPISLION)
  {
    reset_array(angles, GYRO_LOOKBACK, 0);
    resetAngle();
  }

  // This is a bad way to adjust speed but for now it works
  move_motors(SPEED_NORMAL - avg_angle, SPEED_NORMAL + avg_angle);

  if (avg_distance < MIN_DISTANCE) {
    state = 1;
    move_motors(0, 0);
    return;
  }

  if (avg_distance < MIN_DISTANCE + 5) {
    move_motors(-SPEED_NORMAL, -SPEED_NORMAL);
    shake_head();
    move_motors(-200, 200);
    delay(random(500, 2500));
  }
}

void following()
{
  int avg_distance = 0;
  int avg_angle = 0;

  update_array(distances, DISTANCE_LOOKBACK, d_i++, getDistance());
  update_array(angles, GYRO_LOOKBACK, g_i++, getAngle());

  avg_distance = get_array_average(distances, DISTANCE_LOOKBACK);
  avg_angle = get_array_average(angles, GYRO_LOOKBACK);

  ledOn(get_following_colour(avg_distance));

  // If the angle get's off we reset so there isn't runaway addition
  if (abs(avg_angle) >= GYRO_EPISLION)
  {
    reset_array(angles, GYRO_LOOKBACK, 0);
    resetAngle();
  }

  if (avg_distance >= MIN_DISTANCE && avg_distance < MAX_DISTANCE)
  {
    int speed = get_speed(avg_distance);
    move_motors(speed, speed);
  }

  if (avg_distance > MAX_DISTANCE)
  {
    move_motors(0, 0);
    state = 2;
  }
}

void searching()
{
  ledOn(CRGB(48, 25, 52));

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  move_motors(-200, 200);
  delay(300);
  move_motors(0, 0);

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  move_motors(200, -200);
  delay(600);
  move_motors(0, 0);

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  centerServo(2000);

  freak_out();

  state = 0;
}


void follow_line()
{
  int left = analogRead(LINE_L);
  int center = analogRead(LINE_C);
  int right = analogRead(LINE_R);

  bool on_center = center < LINE_THRESHOLD;
  bool on_left = left < LINE_THRESHOLD;
  bool on_right = right < LINE_THRESHOLD;

  move_motors(0, 0);

  if (on_center && on_left && on_right) {
    move_motors(SPEED_NORMAL, SPEED_NORMAL);
    print("motor", "path", 0);
    // move_motors(SPEED_NORMAL, SPEED_NORMAL);
  }

  else if (on_left && !on_right) {
    move_motors(SPEED_NORMAL, -SPEED_NORMAL);
    print("motor", "path", -1);
    // move_motors(-SPEED_TURN, SPEED_TURN);
  }

  else if (on_right && !on_left) {
    move_motors(-SPEED_NORMAL, SPEED_NORMAL);
    print("motor", "path", 1);
    // move_motors(-SPEED_TURN, SPEED_TURN);
  }

  if (!on_center && !on_left && !on_right) {
    state = 0;
  }
}

// STATES
// -1 TESTING PURPOSES (I'm sick and tired of the motor noises)
// 0 ROAMING
// 1 FOLLOWING
// 2 SEARCHING
// 3 FOLLOW LINE

void loop()
{
  updateGyroAngle();

  switch (state)
  {
    case -1:
      break;

    case 0:
      roaming();
      break;
    
    case 1:
      following();
      break;
    
    case 2:
      searching();
      break;
    
    case 3:
      follow_line();
      break;

    default:
      break;
  }
}
