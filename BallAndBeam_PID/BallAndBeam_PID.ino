#include <ESP32Servo.h>

// ─── Pin Definitions ───
const int SENSOR_PIN = 27;
const int SERVO_PIN  = 26;

Servo myservo;

// ─── Timing ───
const int PERIOD = 40;           // 40ms = 25 Hz (matched to IR sensor settling time)
unsigned long lastTime;

// ─── PID Gains (time-independent, dt in seconds) ───
// Gains output DIRECTLY in servo-degrees (no map() scaling).
// Equivalent to your original working gains after accounting
// for the removed map(-150,150,40,125) which scaled by 0.283x.
float kp = 6.0;    // degrees per cm of error
float ki = 1.1;    // degrees per (cm·s) of accumulated error
float kd = 4.0;    // degrees per (cm/s) of error rate  (damping)

// ─── Setpoint ───
float distance_setpoint = 15.0;  // cm

// ─── PID State ───
float distance = 0.0;
float distance_error = 0.0;
float distance_previous_error = 0.0;
float PID_p = 0.0;
float PID_i = 0.0;
float PID_d = 0.0;
float PID_total = 0.0;
float derivative_filtered = 0.0;

// ─── Servo State ───
float previousServoAngle = 90.0;

// ─── Filter Coefficients ───
const float ALPHA_SENSOR     = 0.7;  // sensor EMA (lower = smoother, more lag)
const float ALPHA_DERIVATIVE = 0.5;  // derivative LPF (lower = smoother)

// ─── Limits ───
const int   SERVO_MIN        = 35;
const int   SERVO_MAX        = 130;
const float INTEGRAL_LIMIT   = 30.0;  // anti-windup clamp (degrees)
// MG-996R moves ~400 deg/s => ~16 deg per 40ms cycle. Cap slightly below that
// so we never command faster than the servo can physically follow, but do NOT
// throttle it (throttling adds phase lag and causes a limit cycle).
const float MAX_STEP_PER_CYCLE = 14.0; // max servo degrees per 40ms cycle

// ─── Gain Scheduling ───
// Adapts gains by distance from setpoint. Multipliers scale the BASE
// gains (kp/ki/kd) that you set live from Python, so manual tuning still works.
//   FAR  (|error| > FAR_THRESHOLD)  : aggressive, push the ball hard
//   NEAR (|error| < NEAR_THRESHOLD) : gentle, avoid overshoot/oscillation
//   between the two thresholds      : linearly blended
bool  gainSchedulingEnabled = true;
const float FAR_THRESHOLD  = 5;   // cm
const float NEAR_THRESHOLD = 1.0;   // cm
const float KP_FAR = 1.1, KP_NEAR = 0.7;    // proportional multipliers
const float KD_FAR = 1.0, KD_NEAR = 0.9;    // derivative: keep strong damping near setpoint
const float KI_FAR = 0.5, KI_NEAR = 1.0;    // integral: low far (avoid windup), full near

// ─── Serial Command Parsing ───
bool   systemRunning = false;
String inputString   = "";

// ─── Operational Mode ───
// 0 = Full PID Control
// 1 = Sensor Sanity Check (servo held level, distance still streamed)
int currentMode = 1;

// ═══════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, 544, 2395);
  myservo.write(90);

  analogReadResolution(12);

  lastTime = millis();
  distance = get_dist_raw();  // seed the EMA filter with a real reading
}

// ═══════════════════════════════════════════════════
void loop() {

  // ── Handle Serial Commands from Python ──
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }

  // ── Main PID Loop (runs every PERIOD ms) ──
  unsigned long now = millis();
  if (now - lastTime >= PERIOD) {

    // 1. Compute real elapsed time in seconds
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 2. Read & filter sensor (EMA)
    float raw = get_dist_raw();
    distance = ALPHA_SENSOR * raw + (1.0 - ALPHA_SENSOR) * distance;

    if (systemRunning) {

      // ===== MODE 1: SENSOR SANITY CHECK =====
      // Servo stays level; just stream the distance so you can verify
      // sensor readings on the plot without the ball moving.
      if (currentMode == 1) {
        myservo.write(90);
        previousServoAngle = 90.0;

        // CSV: Distance,Setpoint,Angle,Kp_eff,Ki_eff,Kd_eff
        Serial.print(distance, 1);      Serial.print(",");
        Serial.print(0.0, 1);           Serial.print(",");   // dummy setpoint
        Serial.print(90);               Serial.print(",");   // stationary servo
        Serial.print(kp, 3);            Serial.print(",");
        Serial.print(ki, 3);            Serial.print(",");
        Serial.println(kd, 3);
        return;   // skip PID for this cycle
      }

      // ===== MODE 0: FULL PID CONTROL =====
      // 3. Error
      distance_error = distance_setpoint - distance;

      // 3b. Gain scheduling: blend multipliers by |error|, then scale base gains
      float kp_eff = kp, ki_eff = ki, kd_eff = kd;
      if (gainSchedulingEnabled) {
        float absErr = abs(distance_error);
        // t = 0 near setpoint, 1 far away (clamped, linear in between)
        float t = (absErr - NEAR_THRESHOLD) / (FAR_THRESHOLD - NEAR_THRESHOLD);
        t = constrain(t, 0.0, 1.0);
        kp_eff = kp * (KP_NEAR + t * (KP_FAR - KP_NEAR));
        kd_eff = kd * (KD_NEAR + t * (KD_FAR - KD_NEAR));
        ki_eff = ki * (KI_NEAR + t * (KI_FAR - KI_NEAR));
      }

      // 4. Proportional
      PID_p = kp_eff * distance_error;

      // 5. Integral with anti-windup clamp
      PID_i += ki_eff * distance_error * dt;
      PID_i = constrain(PID_i, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      // 6. Derivative with low-pass filter
      float raw_derivative = kd_eff * (distance_error - distance_previous_error) / dt;
      derivative_filtered = ALPHA_DERIVATIVE * raw_derivative
                          + (1.0 - ALPHA_DERIVATIVE) * derivative_filtered;
      PID_d = derivative_filtered;

      // 7. Total PID output (in degrees offset from level)
      PID_total = PID_p + PID_i + PID_d;

      // 8. Convert to servo angle (90° = beam level)
      float servoAngle = 90.0 + PID_total;

      // 9. Rate-limit: prevent commanding jumps the servo can't physically achieve
      float servoStep = servoAngle - previousServoAngle;
      servoStep = constrain(servoStep, -MAX_STEP_PER_CYCLE, MAX_STEP_PER_CYCLE);
      servoAngle = previousServoAngle + servoStep;

      // 10. Hard-limit to mechanical range
      servoAngle = constrain(servoAngle, (float)SERVO_MIN, (float)SERVO_MAX);
      previousServoAngle = servoAngle;

      // 11. Command servo
      myservo.write((int)servoAngle);

      // 12. Store previous error
      distance_previous_error = distance_error;

      // 13. Send CSV data to Python: Distance,Setpoint,Angle,Kp_eff,Ki_eff,Kd_eff
      Serial.print(distance, 1);          Serial.print(",");
      Serial.print(distance_setpoint, 1); Serial.print(",");
      Serial.print((int)servoAngle);      Serial.print(",");
      Serial.print(kp_eff, 3);            Serial.print(",");
      Serial.print(ki_eff, 3);            Serial.print(",");
      Serial.println(kd_eff, 3);
    }
  }
}

// ═══════════════════════════════════════════════════
// Read raw distance from IR sensor (3-sample average → voltage → cm)
float get_dist_raw() {
  long sum = 0;
  for (int i = 0; i < 3; i++) {
    sum += analogRead(SENSOR_PIN);
  }
  float volts = (sum / 3.0) * (3.3 / 4095.0);
  return constrain(29.988 * pow(volts, -1.173), 10.0, 80.0);
}

// ═══════════════════════════════════════════════════
// Parse serial commands: START, STOP, SET <val>, KP <val>, KI <val>, KD <val>
void processCommand(String cmd) {
  cmd.trim();
  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd == "START") {
    // Reset PID state for a clean start
    PID_i = 0.0;
    derivative_filtered = 0.0;
    distance_previous_error = 0.0;
    previousServoAngle = 90.0;
    systemRunning = true;
  }
  else if (upperCmd == "STOP") {
    systemRunning = false;
    myservo.write(90);            // return beam to level
    previousServoAngle = 90.0;
  }
  else if (upperCmd.startsWith("SET ")) {
    distance_setpoint = cmd.substring(4).toFloat();
  }
  else if (upperCmd.startsWith("KP ")) {
    kp = cmd.substring(3).toFloat();
  }
  else if (upperCmd.startsWith("KI ")) {
    ki = cmd.substring(3).toFloat();
  }
  else if (upperCmd.startsWith("KD ")) {
    kd = cmd.substring(3).toFloat();
  }
  else if (upperCmd == "SCHED ON") {
    gainSchedulingEnabled = true;
  }
  else if (upperCmd == "SCHED OFF") {
    gainSchedulingEnabled = false;
  }
  else if (upperCmd == "MODE 0") {
    currentMode = 0;
    // Reset PID state so control resumes cleanly
    PID_i = 0.0;
    derivative_filtered = 0.0;
    distance_previous_error = 0.0;
    previousServoAngle = 90.0;
  }
  else if (upperCmd == "MODE 1") {
    currentMode = 1;
    myservo.write(90);   // hold beam level
    previousServoAngle = 90.0;
  }
}
