//("N20" motor: 407.7 incs per rvolution)
//"(GA25-370" MOTOR: L=204.06 R=204.1)#include <Servo.h>
// ---- Servo ----
#include <Servo.h>
Servo RollServo, PitchServo, GrippServo;
const float CPR_L = 204.06;
const float CPR_R = 204.1;
const float wheel_radius = 0.0325, wheel_base = 0.250;   // meters 
// Angles offset
#define ROLL_ZERO 84 //nkaber etdour lel port nte3 battery 44 84* 124 (Grp20, pitch74: 0 180)
#define PITCH_ZERO 73 //tkaber tatle3  65 73* 100/ 140 170
#define GRIPPER_ZERO 103 //nsagher tsaker  103* 80->5.9cm 20
#define GrippAngle_closed_max 20


// ---- Motors ----
#define IN1R A3      // right
#define IN2R A2
#define PinPWM_R 6
#define IN1L A0      // left
#define IN2L A1
#define PinPWM_L 5
int16_t target_rpm_L = 0;
int16_t target_rpm_R = 0;
// ----- Gripper Variables -----
uint8_t rollAngle = ROLL_ZERO; 
uint8_t pitchAngle = PITCH_ZERO; 
float grippAngle = GRIPPER_ZERO  ; 
bool close_gripper = 0; 
bool gripper_closed = 0;
float GrippPas = 0.8; 
// ---- Encoders ----
volatile int32_t encR = 0;
volatile int32_t encL = 0;
#define ENC_A_R 3  // interrupt pin //Right
#define ENC_B_R 7
#define ENC_A_L 2  // interrupt pin //Left
#define ENC_B_L 4
unsigned long last_time = 0;
float measured_rpm_R = 0;
float measured_rpm_L = 0;
float errorL = 0;
float prev_error1L = 0;
float prev_error2L = 0;
float errorR = 0;
float prev_error1R = 0;
float prev_error2R = 0;
float pwmL = 0;
float pwmR = 0;
float pwmL1 = 0;
float pwmR1 = 0;

//PID
const int Tm = 20; //ms
const float T = Tm/1000.0f; //s
const float Kp = 0.4 , Ki=2.5 , Kd = 0.003; //for Tm=20ms with filter

// Low-pass filter for RPM measurement
float measured_rpm_filt_L = 0;
float measured_rpm_filt_R = 0;
const float tau = 0.18;   // filter time constant
const float alpha = T / (tau + T);

//mpu6050
#include <Wire.h>
#define MPU_ADDR 0x68
// ----- IMU -----
float gyroZ_offset = 0.0;
float measured_w = 0.0;   // rad/s

// ----- Yaw Rate PID -----
float error_w = 0;
float prev_error_w = 0;
float integral_w = 0;
float Kp_w = 8, Ki_w = 2, Kd_w = 1;

// ---- Odometry ----
float x_pos = 0.0f;
float y_pos = 0.0f;
float theta = 0.0f;
float omega_robot = 0.0f;


void setup() {
  Serial.begin(115200);

  // IMU
  Wire.begin();
  mpu_init();
  delay(2000);
  // ----- Gyro Z calibration -----
  for (int i = 0; i < 1000; i++) {
    gyroZ_offset += readGyroZ();
    delay(2);
  }
  gyroZ_offset /= 1000.0;

  // Servos
  RollServo.attach(8);
  PitchServo.attach(9);
  GrippServo.attach(10);

  // Motors
  pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(IN1R, OUTPUT); pinMode(IN2R, OUTPUT);
  pinMode(PinPWM_L, OUTPUT); pinMode(PinPWM_R, OUTPUT);
  motorL(0);
  motorR(0);
  //brakeL();
  //brakeR();
  RollServo.write(ROLL_ZERO);  
  PitchServo.write(PITCH_ZERO);  
  GrippServo.write(GRIPPER_ZERO); 

  // Encoders
  pinMode(ENC_A_L, INPUT);
  pinMode(ENC_B_L, INPUT);
  pinMode(ENC_A_R, INPUT);
  pinMode(ENC_B_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_L), encL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_R), encR_ISR, CHANGE);
  delay(2000);
}



void loop() {
  // Expecting 13 byte packet: S cmd_v cmd_w rollAngle pitchAngle close_gripper E
  static float cmd_v = 0.0;
  static float cmd_w = 0.0;
  float cmd_v_serial = 0.0;
  float cmd_w_serial = 0.0;
  uint8_t rollAngle_serial = ROLL_ZERO;
  uint8_t pitchAngle_serial = PITCH_ZERO;
  bool close_gripper_serial = 0; 
  while (Serial.available() >= 13) {
      if (Serial.read() == 'S') {
          Serial.readBytes((char*)&cmd_v_serial, 4);
          Serial.readBytes((char*)&cmd_w_serial, 4);
          rollAngle_serial  = Serial.read();
          pitchAngle_serial = Serial.read();
          close_gripper_serial = Serial.read();
          if (Serial.read() == 'E') {
            cmd_v = cmd_v_serial;
            cmd_w = cmd_w_serial;
            rollAngle = rollAngle_serial;
            pitchAngle = pitchAngle_serial;
            close_gripper = close_gripper_serial;
          }
      }
  }
  // If both wheels target is zero → reset PID completely
  if (cmd_v==0 && cmd_w==0) {

      // Reset LEFT PID memory
      pwmL = 0;
      pwmL1 = 0;
      errorL = 0;
      prev_error1L = 0;
      prev_error2L = 0;

      // Reset RIGHT PID memory
      pwmR = 0;
      pwmR1 = 0;
      errorR = 0;
      prev_error1R = 0;
      prev_error2R = 0;

      // Reset filter measurements
      measured_rpm_filt_L = 0;
      measured_rpm_filt_R = 0;

      // Reset yaw rate PID memory
      error_w = 0;
      prev_error_w = 0;
      integral_w = 0;

      // Update
      motorL(0);
      motorR(0);
      RollServo.write(rollAngle);
      PitchServo.write(pitchAngle);
      GrippServo.write(grippAngle);
  }

  // IMU UPDATE 
  float gyroZ = readGyroZ();
  gyroZ -= gyroZ_offset;
  measured_w = gyroZ * PI / 180.0;  // rad/s

  // YAW RATE PID
  error_w = cmd_w - measured_w;
  integral_w += error_w * T;
  float derivative_w = (error_w - prev_error_w) / T;
  float yaw_output =
        Kp_w * error_w
      + Ki_w * integral_w
      + Kd_w * derivative_w;
  prev_error_w = error_w;
  float cmd_w_corrected = yaw_output;

  // Calculate target RPMs
  //float v_l = cmd_v - (cmd_w * wheel_base / 2.0);
  //float v_r = cmd_v + (cmd_w * wheel_base / 2.0);
  float v_l = cmd_v - (cmd_w_corrected * wheel_base / 2.0);
  float v_r = cmd_v + (cmd_w_corrected * wheel_base / 2.0);
  target_rpm_L = (v_l / (2.0 * PI * wheel_radius)) * 60.0;
  target_rpm_R = (v_r / (2.0 * PI * wheel_radius)) * 60.0;
      
  unsigned long currentMillis = millis();
  if (currentMillis - last_time >= Tm) {
      last_time = currentMillis;
      static int32_t prev_encL = 0, prev_encR = 0;
      int32_t current_encL, current_encR;
      noInterrupts();
      current_encL = encL;
      current_encR = encR;
      interrupts();
      int32_t encL_per_loop = current_encL - prev_encL;
      int32_t encR_per_loop = current_encR - prev_encR;
      prev_encL = current_encL;
      prev_encR = current_encR;

      //calculate rpm measured
      measured_rpm_L = (encL_per_loop * 60.0f) / (CPR_L * T);
      measured_rpm_R = (encR_per_loop * 60.0f) / (CPR_R * T);

      // Low-pass filter
      measured_rpm_filt_L = measured_rpm_filt_L + alpha * (measured_rpm_L - measured_rpm_filt_L);
      measured_rpm_L = measured_rpm_filt_L;
      measured_rpm_filt_R = measured_rpm_filt_R + alpha * (measured_rpm_R - measured_rpm_filt_R);
      measured_rpm_R = measured_rpm_filt_R;
    
      //PID_L
      errorL = target_rpm_L - measured_rpm_L;
      pwmL = pwmL1 + (Kp + Kd/T)*errorL + (-Kp + Ki*T - 2*Kd/T)*prev_error1L + (Kd/T)*prev_error2L;
      pwmL1 = pwmL;
      prev_error2L = prev_error1L;
      prev_error1L = errorL;
      
      //PID_R
      errorR = target_rpm_R - measured_rpm_R;
      pwmR = pwmR1 + (Kp + Kd/T)*errorR + (-Kp + Ki*T - 2*Kd/T)*prev_error1R + (Kd/T)*prev_error2R;
      pwmR1 = pwmR;
      prev_error2R = prev_error1R;
      prev_error1R = errorR;
      
      // Update motors
      if ((pwmL < 30) && (pwmL > 10)) pwmL = 30; 
      if ((pwmL < -10) && (pwmL > -30)) pwmL = -30; 
      if ((pwmL < 10) && (pwmL > -10)) pwmL = 0; 
      if ((pwmR < 30) && (pwmR > 10)) pwmR = 30; 
      if ((pwmR < -10) && (pwmR > -30)) pwmR = -30; 
      if ((pwmR < 10) && (pwmR > -10)) pwmR = 0; 
      //if ((pwmL < 30) && (pwmL > -30)) pwmL = 0; // Deadzone
      //if ((pwmR < 30) && (pwmR > -30)) pwmR = 0; // Deadzone
      pwmL=constrain(pwmL, -500, 500);
      pwmR=constrain(pwmR, -500, 500);
      pwmL=pwmL*(255.0/500.0);
      pwmR=pwmR*(255.0/500.0);
      pwmL=constrain(pwmL, -200, 200);
      pwmR=constrain(pwmR, -200, 200);
      motorL((int)pwmL);
      motorR((int)pwmR);

      // update gripper state
      int clicker = analogRead(A6);
      // ----- CLOSED -----
      if ((clicker >= 500) || (grippAngle <= GrippAngle_closed_max)) {
          //if (gripper_closed==0) grippAngle +=1; // stop vibration
          gripper_closed = 1;
      }
      // ----- OPENED -----
      if (grippAngle >= GRIPPER_ZERO  ) gripper_closed = 0;
      // ----- CLOSING -----
      if ((gripper_closed == 0) && (close_gripper == 1)) grippAngle -= GrippPas;
      // ----- OPENING -----
      if ((gripper_closed == 1) && (close_gripper == 0)) grippAngle += GrippPas; 
      // ---- avoid over opening and closing ----
      if (grippAngle < GrippAngle_closed_max) grippAngle = GrippAngle_closed_max;
      if (grippAngle > GRIPPER_ZERO  ) grippAngle = GRIPPER_ZERO  ;
      
      // Update servos
      RollServo.write(rollAngle);
      PitchServo.write(pitchAngle);
      GrippServo.write(grippAngle);
      
      // ---- Send Feedback ----
      float posL = (current_encL * 2.0f * PI) / CPR_L; // rad
      float posR = (current_encR * 2.0f * PI) / CPR_R; // rad
      float velL = measured_rpm_L * 2.0f * PI / 60.0f; // rad/s
      float velR = measured_rpm_R * 2.0f * PI / 60.0f; // rad/s

      float v = (velL + velR) * wheel_radius / 2.0f;
      //omega_robot = (velR - velL) * wheel_radius / wheel_base;
      theta += measured_w * T;
      x_pos += v * T * cos(theta);
      y_pos += v * T * sin(theta);
      if (theta > PI) theta -= 2.0f * PI;
      if (theta < -PI) theta += 2.0f * PI;

      Serial.write('F');
      Serial.write((uint8_t*)&posL, 4);
      Serial.write((uint8_t*)&posR, 4);
      Serial.write((uint8_t*)&velL, 4);
      Serial.write((uint8_t*)&velR, 4);
      Serial.write((uint8_t*)&x_pos, 4);
      Serial.write((uint8_t*)&y_pos, 4);
      Serial.write((uint8_t*)&theta, 4);
      Serial.write((uint8_t*)&measured_w, 4);
      Serial.write((uint8_t)grippAngle); 
      Serial.write(gripper_closed);
      Serial.write('E');

  }
}

// IMU
void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // Power management
  Wire.write(0);      // Wake up MPU6050
  Wire.endTransmission(true);
  // Set gyro full scale ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);   // Gyro Z high byte
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  // 131 LSB per deg/s for ±250 dps
  return (float)gz / 131.0;
}



// Encoder ISR
void encL_ISR() { if (digitalRead(ENC_A_L) == digitalRead(ENC_B_L)) encL++; else encL--; }
void encR_ISR() { if (digitalRead(ENC_A_R) == digitalRead(ENC_B_R)) encR--; else encR++; }

// Motors
void motorR(int speed_) {
  if (speed_ > 0) {
    // Forward
    digitalWrite(IN1R, LOW);
    digitalWrite(IN2R, HIGH);
    analogWrite(PinPWM_R, speed_);
  } else if (speed_ < 0) {
    // Backward
    digitalWrite(IN1R, HIGH);
    digitalWrite(IN2R, LOW);
    analogWrite(PinPWM_R, -speed_);
  } else {
    // Stop
    analogWrite(PinPWM_R, 0);
    digitalWrite(IN1R, LOW);
    digitalWrite(IN2R, LOW);
  }
}
void motorL(int speed_) {
  if (speed_ > 0) {
    // Forward
    digitalWrite(IN1L, HIGH);
    digitalWrite(IN2L, LOW);
    analogWrite(PinPWM_L, speed_);
  } else if (speed_ < 0) {
    // Backward
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, HIGH);
    analogWrite(PinPWM_L, -speed_);
  } else {
    // Stop
    analogWrite(PinPWM_L, 0);
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
  }
}
void brakeR() {
  analogWrite(PinPWM_R, 0);
  digitalWrite(IN1R, HIGH);
  digitalWrite(IN2R, HIGH);
}
void brakeL() {
  analogWrite(PinPWM_L, 0);
  digitalWrite(IN1L, HIGH);
  digitalWrite(IN2L, HIGH);
}


