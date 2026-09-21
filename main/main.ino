#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ==================== Pin Mappings ====================
// Joystick Analog Inputs (ADC1)
#define JOY_X_PIN      34   // Joystick X-Axis -> Controls Servo Angle
#define JOY_Y_PIN      35   // Joystick Y-Axis -> Controls Stepper Motion

// Servo Output Pin
#define SERVO_PIN      26   // Servo PWM Control Signal Pin

// Stepper Motor ULN2003 Driver Pins
#define IN1            13
#define IN2            12
#define IN3            14
#define IN4            27

// I2C Bus Pins
#define I2C_SDA        21
#define I2C_SCL        22

// ==================== Stepper Parameters ====================
// 28BYJ-48 Stepper Motor: 2048 steps per revolution in 8-step half-step mode
const long STEPS_PER_REV = 2048;
const long MAX_STEPS = STEPS_PER_REV * 2; // Limit: +/- 2 full revolutions (+/- 4096 steps)

// 8-Step Half-Step Sequence Matrix for Smooth Acceleration & Rotation
const int stepMatrix[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// ==================== Object Declarations ====================
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== System Variables ====================
int currentServoAngle = 90;
long currentStepCount = 0;
int stepIndex = 0;

// Non-Blocking Timing Variables
unsigned long lastStepperStepTime = 0;
unsigned long lastLCDUpdateTime = 0;
const int STEP_INTERVAL_US = 2500; // Step pulse delay in microseconds (controls step speed)
const int LCD_INTERVAL_MS = 50;    // Telemetry refresh rate in milliseconds

// ==================== Helper Functions ====================

// Output half-step excitation pattern to ULN2003 driver
void writeStep(int step) {
  digitalWrite(IN1, stepMatrix[step][0]);
  digitalWrite(IN2, stepMatrix[step][1]);
  digitalWrite(IN3, stepMatrix[step][2]);
  digitalWrite(IN4, stepMatrix[step][3]);
}

// De-energize all coils to save power, reduce thermal build-up, and free up COM port
void stopStepper() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ==================== Setup Function ====================
void setup() {
  Serial.begin(115200);

  // Configure ADC attenuation (0V to 3.3V full-scale range)
  analogSetAttenuation(ADC_11db);

  // Configure Stepper Driver GPIO Output Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopStepper(); // Ensure driver is de-energized on startup

  // Initialize Servo PWM Channel using ESP32 PWM Allocator
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50); // Standard 50Hz Servo Frequency
  myServo.attach(SERVO_PIN, 500, 2400); // 500us - 2400us pulse width limits
  myServo.write(currentServoAngle);

  // Initialize I2C LCD Display
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  // Display Boot Screen
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Arm System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1500);
  lcd.clear();
}

// ==================== Main Loop ====================
void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  // ----------------------------------------------------
  // 1. Process Servo Motion (Joystick X-Axis)
  // ----------------------------------------------------
  int rawX = analogRead(JOY_X_PIN);
  int targetAngle = map(rawX, 150, 3900, 0, 180);
  targetAngle = constrain(targetAngle, 0, 180);

  // Update Servo position only if position delta exceeds noise threshold
  if (abs(targetAngle - currentServoAngle) > 1) {
    currentServoAngle = targetAngle;
    myServo.write(currentServoAngle);
  }

  // ----------------------------------------------------
  // 2. Process Stepper Motion (Joystick Y-Axis)
  // ----------------------------------------------------
  int rawY = analogRead(JOY_Y_PIN);

  if (currentMicros - lastStepperStepTime >= STEP_INTERVAL_US) {
    // Deadzone Window: 1300 to 2700 (prevents automatic drift)
    if (rawY > 2700) { // Push UP -> Clockwise Step Generation
      if (currentStepCount < MAX_STEPS) {
        stepIndex = (stepIndex + 1) % 8;
        writeStep(stepIndex);
        currentStepCount++;
        lastStepperStepTime = currentMicros;
      }
    } else if (rawY < 1300) { // Push DOWN -> Counter-Clockwise Step Generation
      if (currentStepCount > -MAX_STEPS) {
        stepIndex = (stepIndex - 1 + 8) % 8;
        writeStep(stepIndex);
        currentStepCount--;
        lastStepperStepTime = currentMicros;
      }
    } else {
      // In center position: cut coil power to prevent thermal build-up and floating steps
      stopStepper();
    }
  }

  // ----------------------------------------------------
  // 3. Non-Blocking LCD Telemetry Display
  // ----------------------------------------------------
  if (currentMillis - lastLCDUpdateTime >= LCD_INTERVAL_MS) {
    lastLCDUpdateTime = currentMillis;

    lcd.setCursor(0, 0);
    lcd.print("Servo: ");
    lcd.print(currentServoAngle);
    lcd.print((char)223); // Degree symbol
    lcd.print("   ");     // Clear residual characters

    lcd.setCursor(0, 1);
    lcd.print("Steps: ");
    lcd.print(currentStepCount);
    lcd.print("   ");     // Clear residual characters
  }
}