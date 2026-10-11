// ESP32 brushed micro quadcopter flight controller (BLE control)
// Hardware: ESP32-WROOM-32, GY-91 (MPU6500) on I2C, 4x AO3400 MOSFET-driven 8520 motors.
//
// Pipeline each loop: read IMU -> complementary filter -> PID (angle mode)
//                     -> X-quad motor mixer -> soft-start ramp -> PWM out.
// Safety: boots disarmed, failsafe disarms on BLE disconnect or 500 ms without a packet,
//         disarm cuts motors instantly (bypasses the ramp).

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Motor pins ----------
// Corner layout (top view). Make sure each physical motor matches its pin.
//   M1(FL)  M2(FR)
//   M3(RL)  M4(RR)
#define M1_PIN 25 // Front-Left
#define M2_PIN 26 // Front-Right
#define M3_PIN 27 // Rear-Left
#define M4_PIN 33 // Rear-Right

// ---------- Arm indicator LED ----------
#define LED_PIN 2 // onboard blue LED: ON when armed, OFF when disarmed

const int PWM_FREQ = 20000;
const int PWM_RES = 8;

// ---------- BLE ----------
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_DEVICE_NAME    "ESP32-Quad-Control"

BLEServer* pServer = nullptr;
BLECharacteristic* pControlChar = nullptr;
bool bleConnected = false;

// 9 bytes, little-endian: matches Python struct '<Bhhhh'
struct __attribute__((packed)) CommandPacket {
  uint8_t arm;      // 0 = disarmed, 1 = armed
  int16_t throttle; // 0..1000
  int16_t pitch;    // -500..500
  int16_t roll;     // -500..500
  int16_t yaw;      // -500..500 (received, not yet mixed in)
};

CommandPacket currentCommand = {0, 0, 0, 0, 0};
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500; // ms

// ---------- BLE callbacks ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("BLE client connected.");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    currentCommand.arm = 0;
    currentCommand.throttle = 0;
    Serial.println("BLE client disconnected - disarming.");
    BLEDevice::startAdvertising();
  }
};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue(); // Arduino String in this core version
    if (v.length() == sizeof(CommandPacket)) {
      memcpy(&currentCommand, v.c_str(), sizeof(CommandPacket));
      lastPacketTime = millis();
    }
  }
};

// ---------- IMU ----------
#define MPU_ADDR 0x68
float gyroXoffset = 0, gyroYoffset = 0;
float pitchAngle = 0, rollAngle = 0;
float pitchOffset = 0, rollOffset = 0; // resting-angle bias measured at boot
unsigned long lastIMUTime;
const float ALPHA = 0.98; // complementary filter: gyro weight

void calibrateGyro() {
  Serial.println("Calibrating gyro... keep still");
  long sumX = 0, sumY = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 4, true);

    int16_t gxSample = Wire.read() << 8 | Wire.read(); // int16_t cast matters (sign extension)
    int16_t gySample = Wire.read() << 8 | Wire.read();
    sumX += gxSample;
    sumY += gySample;
    delay(2);
  }
  gyroXoffset = sumX / (float)samples;
  gyroYoffset = sumY / (float)samples;
  Serial.print("Gyro calibrated. Offsets - X: "); Serial.print(gyroXoffset);
  Serial.print("  Y: "); Serial.println(gyroYoffset);
}

// Whatever tilt the board has at boot becomes (0, 0).
void calibrateAngleOffset() {
  Serial.println("Calibrating baseline angle... keep still");
  float sumPitch = 0, sumRoll = 0;
  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    int16_t axRaw = Wire.read() << 8 | Wire.read();
    int16_t ayRaw = Wire.read() << 8 | Wire.read();
    int16_t azRaw = Wire.read() << 8 | Wire.read();

    float ax = axRaw / 16384.0;
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    sumPitch += atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    sumRoll  += atan2(ay, az) * 180.0 / PI;
    delay(5);
  }

  pitchOffset = sumPitch / samples;
  rollOffset  = sumRoll / samples;
  pitchAngle = 0;
  rollAngle = 0;

  Serial.print("Baseline angle set - Pitch offset: "); Serial.print(pitchOffset);
  Serial.print("  Roll offset: "); Serial.println(rollOffset);
}

void updateIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t axRaw = Wire.read() << 8 | Wire.read();
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  int16_t gyRaw = Wire.read() << 8 | Wire.read();

  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;
  float gx = (gxRaw - gyroXoffset) / 131.0;
  float gy = (gyRaw - gyroYoffset) / 131.0;

  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0;
  lastIMUTime = now;
  if (dt > 0.05 || dt <= 0) dt = 0.01; // guard against abnormal gaps

  float accelPitch = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI) - pitchOffset;
  float accelRoll  = (atan2(ay, az) * 180.0 / PI) - rollOffset;

  pitchAngle = ALPHA * (pitchAngle + gy * dt) + (1 - ALPHA) * accelPitch;
  rollAngle  = ALPHA * (rollAngle  + gx * dt) + (1 - ALPHA) * accelRoll;
}

// ---------- PID (angle mode) ----------
float Kp_pitch = 2.0, Ki_pitch = 0.0, Kd_pitch = 0.0;
float Kp_roll  = 2.0, Ki_roll  = 0.0, Kd_roll  = 0.0;

float pitchIntegral = 0, rollIntegral = 0;
float lastPitchError = 0, lastRollError = 0;
float pitchOutput = 0, rollOutput = 0;

const float MAX_ANGLE = 30.0;    // full stick = this many degrees
const float MAX_INTEGRAL = 50.0;
const float MAX_OUTPUT = 100.0;

unsigned long lastPIDTime;

void computePID() {
  unsigned long now = micros();
  float dt = (now - lastPIDTime) / 1000000.0;
  lastPIDTime = now;
  if (dt > 0.05 || dt <= 0) dt = 0.01;

  float desiredPitch = (currentCommand.pitch / 500.0) * MAX_ANGLE;
  float desiredRoll  = (currentCommand.roll  / 500.0) * MAX_ANGLE;

  float pitchError = desiredPitch - pitchAngle;
  float rollError  = desiredRoll  - rollAngle;

  pitchIntegral += pitchError * dt;
  rollIntegral  += rollError  * dt;
  pitchIntegral = constrain(pitchIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);
  rollIntegral  = constrain(rollIntegral,  -MAX_INTEGRAL, MAX_INTEGRAL);

  float pitchDerivative = (pitchError - lastPitchError) / dt;
  float rollDerivative  = (rollError  - lastRollError)  / dt;
  lastPitchError = pitchError;
  lastRollError  = rollError;

  pitchOutput = (Kp_pitch * pitchError) + (Ki_pitch * pitchIntegral) + (Kd_pitch * pitchDerivative);
  rollOutput  = (Kp_roll  * rollError)  + (Ki_roll  * rollIntegral)  + (Kd_roll  * rollDerivative);

  pitchOutput = constrain(pitchOutput, -MAX_OUTPUT, MAX_OUTPUT);
  rollOutput  = constrain(rollOutput,  -MAX_OUTPUT, MAX_OUTPUT);

  if (currentCommand.arm == 0) {
    pitchIntegral = 0;
    rollIntegral = 0;
  }
}

// ---------- Motor mixing (with soft-start ramp) ----------
int m1_speed = 0, m2_speed = 0, m3_speed = 0, m4_speed = 0;
float curM1 = 0, curM2 = 0, curM3 = 0, curM4 = 0;

const int MIN_ARMED_THROTTLE = 20;
const int MAX_MOTOR_OUTPUT = 255;
const int THROTTLE_ACTIVE_THRESHOLD = 30; // below this (0-1000 scale) motors stay off
const float RAMP_STEP = 4.0; // max PWM change per loop (~10 ms); gentle on the battery

float rampToward(float current, float target) {
  if (current < target) {
    current += RAMP_STEP;
    if (current > target) current = target;
  } else if (current > target) {
    current -= RAMP_STEP;
    if (current < target) current = target;
  }
  return current;
}

void mixMotors() {
  float throttleScaled = (currentCommand.throttle / 1000.0) * 255.0;
  float yawOutput = 0; // yaw not mixed in yet

  float fm1, fm2, fm3, fm4;
  bool active = (currentCommand.arm == 1 && currentCommand.throttle > THROTTLE_ACTIVE_THRESHOLD);

  if (active) {
    fm1 = throttleScaled + pitchOutput + rollOutput - yawOutput;
    fm2 = throttleScaled + pitchOutput - rollOutput + yawOutput;
    fm3 = throttleScaled - pitchOutput + rollOutput + yawOutput;
    fm4 = throttleScaled - pitchOutput - rollOutput - yawOutput;

    fm1 = max(fm1, (float)MIN_ARMED_THROTTLE);
    fm2 = max(fm2, (float)MIN_ARMED_THROTTLE);
    fm3 = max(fm3, (float)MIN_ARMED_THROTTLE);
    fm4 = max(fm4, (float)MIN_ARMED_THROTTLE);

    fm1 = constrain(fm1, 0, (float)MAX_MOTOR_OUTPUT);
    fm2 = constrain(fm2, 0, (float)MAX_MOTOR_OUTPUT);
    fm3 = constrain(fm3, 0, (float)MAX_MOTOR_OUTPUT);
    fm4 = constrain(fm4, 0, (float)MAX_MOTOR_OUTPUT);
  } else {
    fm1 = fm2 = fm3 = fm4 = 0;
  }

  if (currentCommand.arm == 0) {
    curM1 = curM2 = curM3 = curM4 = 0; // disarm cuts instantly, no ramp
  } else {
    curM1 = rampToward(curM1, fm1);
    curM2 = rampToward(curM2, fm2);
    curM3 = rampToward(curM3, fm3);
    curM4 = rampToward(curM4, fm4);
  }

  m1_speed = (int)curM1;
  m2_speed = (int)curM2;
  m3_speed = (int)curM3;
  m4_speed = (int)curM4;

  ledcWrite(M1_PIN, m1_speed);
  ledcWrite(M2_PIN, m2_speed);
  ledcWrite(M3_PIN, m3_speed);
  ledcWrite(M4_PIN, m4_speed);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(21, 22); // SDA, SCL
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1: wake the MPU
  Wire.write(0);
  Wire.endTransmission(true);
  delay(300);

  calibrateGyro();        // keep the board still
  calibrateAngleOffset(); // start position becomes 0/0

  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(M1_PIN, 0);
  ledcWrite(M2_PIN, 0);
  ledcWrite(M3_PIN, 0);
  ledcWrite(M4_PIN, 0);

  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* service = pServer->createService(SERVICE_UUID);
  pControlChar = service->createCharacteristic(
      CONTROL_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pControlChar->setCallbacks(new ControlCallbacks());
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE advertising started.");

  lastIMUTime = micros();
  lastPIDTime = micros();
  lastPacketTime = millis();
}

// ---------- Main loop ----------
void loop() {
  updateIMU();

  // While disarmed, bleed the angle estimate back to 0 (absorbs residual gyro bias)
  if (currentCommand.arm == 0) {
    pitchAngle *= 0.95;
    rollAngle *= 0.95;
  }

  if (!bleConnected || millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
    currentCommand.arm = 0;
    currentCommand.throttle = 0;
  }

  digitalWrite(LED_PIN, currentCommand.arm == 1 ? HIGH : LOW);

  computePID();
  mixMotors();

  Serial.print("Arm:"); Serial.print(currentCommand.arm);
  Serial.print(" Thr:"); Serial.print(currentCommand.throttle);
  Serial.print(" P:"); Serial.print(pitchAngle, 1);
  Serial.print(" R:"); Serial.print(rollAngle, 1);
  Serial.print(" | M1:"); Serial.print(m1_speed);
  Serial.print(" M2:"); Serial.print(m2_speed);
  Serial.print(" M3:"); Serial.print(m3_speed);
  Serial.print(" M4:"); Serial.println(m4_speed);

  delay(10);
}
