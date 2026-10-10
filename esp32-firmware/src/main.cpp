// Robot dog firmware: streams sensor data to the PC over Wi-Fi (UDP) and runs
// the commands the PC sends back. The ML runs on the PC (stuck-leg/live.py).
//
// Protocol (JSON over UDP, port UDP_PORT):
//   PC -> robot   {"cmd":"hello"}                 discovery (broadcast); robot replies {"hello":"robotdog",...}
//                 {"cmd":"hb"}                    heartbeat, every 100 ms; silence > 300 ms = motors off
//                 {"cmd":"gait","on":1,"freq":1.5,"amp":25,"turn":0}   start/stop walking; freq, amp, turn optional
//                                                 turn -1..1: negative = turn left, positive = turn right
//                 {"cmd":"stop"}                  motors off
//                 {"cmd":"unstick","leg":"FL"}    wiggle that leg for UNSTICK_MS, then resume walking
//                 {"cmd":"resume"}                end an unstick early
//                 {"cmd":"detach","leg":"FL"}     motor off + latch servo releases the leg
//                 {"cmd":"attach","leg":"FL"}     latch servo locks, leg rejoins the gait
//                 {"cmd":"zero"}                  current leg angles become 0 degrees
//                 {"cmd":"sim_jam","leg":"FL","s":1.0}  SIM_SENSORS only: jam a leg (leg null = free)
//   robot -> PC   {"t":ms,"imu":[ax,ay,az,gx,gy,gz],"tof":[4],"cur":[4],"enc":[4],"cmd":[4],
//                  "ph":phase,"gait":0/1,"att":[4],"sim":0/1,"jam":-1..3}   at TELEMETRY_HZ

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>
#include "config.h"

#if !SIM_SENSORS
#include <Wire.h>
#if HAS_IMU
#include <Adafruit_MPU6050.h>
#endif
#if HAS_TOF
#include <VL53L0X.h>
#endif
#if HAS_CURRENT
#include <Adafruit_INA219.h>
#endif
#if HAS_ENCODERS
#include <ESP32Encoder.h>
#endif
#if HAS_PCA9685
#include <Adafruit_PWMServoDriver.h>
#endif
#endif

static const char* LEG_NAMES[4] = {"FL", "FR", "RL", "RR"};
static const float TWO_PI_F = 6.28318530718f;

// ====================================================================== shared state
// Written by the command handler (main loop) and the control task; guarded by `mux`.
struct State {
  bool gaitOn = false;
  float gaitFreq = GAIT_FREQ_HZ;
  float gaitAmp = GAIT_AMP_DEG;
  float gaitTurn = 0.0f;            // -1..1; scales left vs right stride
  float phase = 0.0f;
  bool attached[4] = {true, true, true, true};
  int unstickLeg = -1;              // -1 = not unsticking
  uint32_t unstickStartMs = 0;
  float unstickCenter = 0.0f;
  float target[4] = {0, 0, 0, 0};   // commanded joint angle, deg
  float enc[4] = {0, 0, 0, 0};      // measured joint angle, deg
  float cur[4] = {0, 0, 0, 0};      // motor current, mA
};
static State S;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Body sensors, read in the main loop at TELEMETRY_HZ.
static float imu[6] = {0, 0, 9.81f, 0, 0, 0};   // m/s^2, deg/s
static float tofMm[4] = {0, 0, 0, 0};

// ====================================================================== networking
static WiFiUDP udp;
static IPAddress pcIP;
static uint16_t pcPort = 0;
static bool havePc = false;
static uint32_t lastPcMs = 0;
static bool apMode = false;

static void startWifi() {
  WiFi.setHostname(HOSTNAME);
  if (!FORCE_AP_MODE) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Joining Wi-Fi \"%s\"", WIFI_SSID);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. Robot IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("Made Wi-Fi network \"%s\" (password %s). Robot IP: %s\n", AP_SSID, AP_PASS,
                  WiFi.softAPIP().toString().c_str());
  }
  WiFi.setSleep(false);   // power saving adds 100+ ms of latency
  if (MDNS.begin(HOSTNAME)) MDNS.addService("robotdog", "udp", UDP_PORT);
  udp.begin(UDP_PORT);
  Serial.printf("Listening on UDP port %d\n", UDP_PORT);
}

static void sendTo(IPAddress ip, uint16_t port, const char* data, int len) {
  udp.beginPacket(ip, port);
  udp.write((const uint8_t*)data, len);
  udp.endPacket();
}

// ====================================================================== motors and servos
#if !SIM_SENSORS
static void motorsBegin() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(2 * i, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(2 * i + 1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(MOTOR_IN1[i], 2 * i);
    ledcAttachPin(MOTOR_IN2[i], 2 * i + 1);
    ledcWrite(2 * i, 0);
    ledcWrite(2 * i + 1, 0);
  }
}

// pwm in [-MAX_PWM, MAX_PWM]; 0 = coast.
static void setMotor(int i, float pwm) {
  pwm *= MOTOR_DIR[i];
  int duty = (int)fminf(fabsf(pwm), (float)MAX_PWM);
  if (duty < 20) duty = 0;   // too small to move the motor; avoid whine
  ledcWrite(2 * i, pwm > 0 ? duty : 0);
  ledcWrite(2 * i + 1, pwm > 0 ? 0 : duty);
}
#endif

#if !SIM_SENSORS && HAS_PCA9685
static Adafruit_PWMServoDriver pca(PCA9685_ADDR);
static bool pcaOk = false;
#endif

static void setServo(int i, int deg) {
#if SIM_SENSORS
  (void)i; (void)deg;
#else
  uint16_t us = (uint16_t)(500 + deg * 2000 / 180);   // 0..180 deg -> 500..2500 us
#if HAS_PCA9685
  if (pcaOk) pca.writeMicroseconds(SERVO_CH[i], us);
#else
  if (SERVO_PIN[i] >= 0) ledcWrite(8 + i, (uint32_t)us * 65535 / 20000);   // 16-bit duty at 50 Hz
#endif
#endif
}

static void servosBegin() {
#if !SIM_SENSORS
#if HAS_PCA9685
  pcaOk = pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(50);
#else
  for (int i = 0; i < 4; i++) {
    if (SERVO_PIN[i] < 0) continue;
    ledcSetup(8 + i, 50, 16);
    ledcAttachPin(SERVO_PIN[i], 8 + i);
  }
#endif
#endif
  for (int i = 0; i < 4; i++) setServo(i, SERVO_LOCK_DEG);
}

// ====================================================================== real sensors
#if !SIM_SENSORS
#if HAS_IMU
static Adafruit_MPU6050 mpu;
static bool mpuOk = false;
#endif
#if HAS_TOF
static VL53L0X tof[4];
static bool tofOk[4] = {false, false, false, false};
static bool tofSingle = false;   // one sensor copied to all four slots
#endif
#if HAS_CURRENT
static Adafruit_INA219 ina[4] = {Adafruit_INA219(INA219_ADDR[0]), Adafruit_INA219(INA219_ADDR[1]),
                                 Adafruit_INA219(INA219_ADDR[2]), Adafruit_INA219(INA219_ADDR[3])};
static bool inaOk[4] = {false, false, false, false};
#endif
#if HAS_ENCODERS
static ESP32Encoder encoder[4];
#endif

#if HAS_TOF
static bool xshutAvailable(int i) {
#if HAS_PCA9685
  (void)i;
  return pcaOk;
#else
  return XSHUT_PIN[i] >= 0;
#endif
}

static void xshut(int i, bool on) {
#if HAS_PCA9685
  if (on) pca.setPWM(XSHUT_CH[i], 4096, 0);   // fully on = high
  else pca.setPWM(XSHUT_CH[i], 0, 4096);      // fully off = low
#else
  pinMode(XSHUT_PIN[i], OUTPUT);
  digitalWrite(XSHUT_PIN[i], on ? HIGH : LOW);
#endif
}

static void startTof(VL53L0X& s) {
  s.setMeasurementTimingBudget(20000);
  s.startContinuous();
}

static void tofBegin() {
  int withXshut = 0;
  for (int i = 0; i < 4; i++) withXshut += xshutAvailable(i);
  if (withXshut == 0) {          // no XSHUT lines: one sensor at its default address
    tof[0].setTimeout(30);
    if (tof[0].init()) {
      startTof(tof[0]);
      tofOk[0] = tofSingle = true;
    }
    return;
  }
  // All sensors share address 0x29 at power-up. Hold them all in reset, then
  // wake one at a time and give each its own address.
  for (int i = 0; i < 4; i++) if (xshutAvailable(i)) xshut(i, false);
  delay(10);
  for (int i = 0; i < 4; i++) {
    if (!xshutAvailable(i)) continue;
    xshut(i, true);
    delay(10);
    tof[i].setTimeout(30);
    if (tof[i].init()) {
      tof[i].setAddress(TOF_ADDR[i]);
      startTof(tof[i]);
      tofOk[i] = true;
    }
  }
}

// Non-blocking: only reads a sensor whose new measurement is ready.
static void readTof(int i) {
  if ((tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  uint16_t mm = tof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  tof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  if (mm < 8190) tofMm[i] = mm;   // 8190/8191 = out of range
}
#endif

static void sensorsBegin() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
#if HAS_IMU
  mpuOk = mpu.begin();
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  Serial.printf("IMU: %s\n", mpuOk ? "ok" : "NOT FOUND");
#endif
#if HAS_CURRENT
  for (int i = 0; i < 4; i++) {
    inaOk[i] = ina[i].begin();
    Serial.printf("Current sensor %s (0x%02X): %s\n", LEG_NAMES[i], INA219_ADDR[i], inaOk[i] ? "ok" : "NOT FOUND");
  }
#endif
#if HAS_ENCODERS
  ESP32Encoder::useInternalWeakPullResistors = puType::up;   // no effect on GPIO 34-39
  for (int i = 0; i < 4; i++) {
    encoder[i].attachFullQuad(ENC_A[i], ENC_B[i]);
    encoder[i].clearCount();
  }
#endif
}

static void readBodySensors() {
#if HAS_IMU
  if (mpuOk) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    imu[0] = a.acceleration.x;
    imu[1] = a.acceleration.y;
    imu[2] = a.acceleration.z;
    imu[3] = g.gyro.x * RAD_TO_DEG;
    imu[4] = g.gyro.y * RAD_TO_DEG;
    imu[5] = g.gyro.z * RAD_TO_DEG;
  }
#endif
#if HAS_TOF
  for (int i = 0; i < 4; i++) if (tofOk[i]) readTof(i);
  if (tofSingle) for (int i = 1; i < 4; i++) tofMm[i] = tofMm[0];
#endif
#if HAS_CURRENT
  float c[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) if (inaOk[i]) c[i] = fabsf(ina[i].getCurrent_mA());
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) S.cur[i] = c[i];
  portEXIT_CRITICAL(&mux);
#endif
}

static void readEncoders(float out[4]) {
#if HAS_ENCODERS
  for (int i = 0; i < 4; i++) out[i] = (float)encoder[i].getCount() * 360.0f / ENC_COUNTS_PER_REV * ENC_DIR[i];
#else
  for (int i = 0; i < 4; i++) out[i] = 0;
#endif
}

static void zeroEncoders() {
#if HAS_ENCODERS
  for (int i = 0; i < 4; i++) encoder[i].clearCount();
#endif
}
#endif  // !SIM_SENSORS

// ====================================================================== simulated robot
// A C++ port of fake_robot.py, stepped at TELEMETRY_HZ, so a model trained on
// fake data recognises jams made on the ESP32 too.
#if SIM_SENSORS
struct Sim {
  float enc[4] = {0, 0, 0, 0};
  float lastCmd[4] = {0, 0, 0, 0};
  float tilt[2] = {0, 0};        // roll, pitch (rad)
  float terrain[2] = {0, 0};
  float bump = 0, turn = 0, turnTarget = 0;
  int jam = -1;                  // jammed leg, -1 = none
  float strength = 1.0f;         // 1 = hard jam, <1 = partial snag
  int unstickSeen = -1;          // leg being wiggled at the last step
  bool freeAfterUnstick = false; // rolled when the unstick command arrived
};
static Sim sim;
static const int CORNER_X[4] = {1, 1, -1, -1};   // +1 = front
static const int CORNER_Y[4] = {1, -1, 1, -1};   // +1 = left

static float urand() { return (float)esp_random() / 4294967296.0f; }   // [0, 1)
static float gauss(float sd) {                                         // Box-Muller
  float u1 = fmaxf(urand(), 1e-7f), u2 = urand();
  return sd * sqrtf(-2.0f * logf(u1)) * cosf(TWO_PI_F * u2);
}

static void simStep(float dt) {
  const float rough = 0.8f, G = 9.81f;
  float cmd[4];
  bool att[4], on;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) { cmd[i] = S.target[i]; att[i] = S.attached[i]; }
  on = S.gaitOn;
  portEXIT_CRITICAL(&mux);

  int bumpLeg = -1;
  if (urand() < 0.02f * rough) { sim.bump = 1.0f; bumpLeg = (int)(urand() * 4) & 3; }
  sim.bump *= 0.85f;
  for (int k = 0; k < 2; k++) {
    sim.terrain[k] = fmaxf(-0.05f, fminf(0.05f, sim.terrain[k] + gauss(0.004f * rough)));
  }
  if (urand() < 0.01f) { const float t[7] = {0, 0, 0, -20, 20, -40, 40}; sim.turnTarget = t[(int)(urand() * 7) % 7]; }
  sim.turn += (sim.turnTarget - sim.turn) * fminf(1.0f, 3 * dt);

  float cur[4], sag[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    float c = cmd[i];
    float dc = (c - sim.lastCmd[i]) / dt;   // deg/s
    sim.lastCmd[i] = c;
    float current;
    if (!att[i] || !on) {                   // motor off: leg stays put, no load
      current = fabsf(gauss(5.0f));
    } else if (i == sim.jam) {
      float follow = 1.0f - sim.strength;
      sim.enc[i] += (c - sim.enc[i]) * fminf(1.0f, 8 * dt) * follow + gauss(0.4f);
      current = (350 + 250 * sim.strength) + gauss(40);
      sag[i] = -7 * sim.strength;
    } else {
      sim.enc[i] += (c - sim.enc[i]) * fminf(1.0f, 15 * dt) + gauss(0.6f * rough);
      float stance = (c < 0) ? 1.0f : 0.0f;  // foot on the ground half the cycle
      current = 120 + 1.2f * fabsf(dc) + 90 * stance + gauss(25 * rough);
    }
    if (i == bumpLeg || (sim.bump > 0.3f && urand() < 0.15f)) current += 220 * sim.bump;
    cur[i] = fmaxf(0.0f, current);
  }

  float tRoll = 0, tPitch = 0, yaw = 0;
  if (sim.jam >= 0 && on) {
    float s = sim.strength;
    tPitch = -0.06f * CORNER_X[sim.jam] * s;
    tRoll = 0.06f * CORNER_Y[sim.jam] * s;
    yaw = 15 * CORNER_Y[sim.jam] * s;
  }
  sim.tilt[0] += (tRoll - sim.tilt[0]) * fminf(1.0f, 4 * dt);
  sim.tilt[1] += (tPitch - sim.tilt[1]) * fminf(1.0f, 4 * dt);
  float roll = sim.tilt[0] + sim.terrain[0], pitch = sim.tilt[1] + sim.terrain[1];
  float ph, turnCmd;
  int unstickNow;
  portENTER_CRITICAL(&mux);
  ph = S.phase;
  turnCmd = S.gaitTurn;
  unstickNow = S.unstickLeg;
  portEXIT_CRITICAL(&mux);
  if (sim.unstickSeen >= 0 && unstickNow < 0) {          // a wiggle just finished: did it come loose?
    if (sim.unstickSeen == sim.jam && sim.freeAfterUnstick) {
      Serial.printf("Sim: wiggling freed %s\n", LEG_NAMES[sim.jam]);
      sim.jam = -1;
    }
    sim.freeAfterUnstick = false;
  }
  sim.unstickSeen = unstickNow;

  float bounce = on ? 0.6f * sinf(2 * TWO_PI_F * ph) : 0.0f;
  float n = 0.25f * rough;
  imu[0] = -G * sinf(pitch) + gauss(n) + 1.5f * sim.bump * gauss(1);
  imu[1] = G * sinf(roll) + gauss(n);
  imu[2] = G * cosf(roll) * cosf(pitch) + bounce + gauss(n);
  imu[3] = gauss(3 * rough) + 40 * (tRoll - sim.tilt[0]);
  imu[4] = gauss(3 * rough) + 40 * (tPitch - sim.tilt[1]);
  imu[5] = yaw + (on ? sim.turn - 30 * turnCmd : 0) + gauss(4 * rough);   // turning right = negative yaw rate

  for (int i = 0; i < 4; i++) {
    float h = 80.0f + 150 * (CORNER_X[i] * sinf(pitch) - CORNER_Y[i] * sinf(roll));
    h += 3 * sinf(TWO_PI_F * (ph + PHASE_OFFSET[i])) + sag[i] + gauss(2.5f * rough);
    if (urand() < 0.01f) h += (urand() < 0.5f ? -1 : 1) * (15 + 25 * urand());   // ToF glitch
    tofMm[i] = h;
  }
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) { S.enc[i] = sim.enc[i]; S.cur[i] = cur[i]; }
  portEXIT_CRITICAL(&mux);
}
#endif  // SIM_SENSORS

// ====================================================================== control loop
// Gait targets (and on real hardware, PD position control) at CONTROL_HZ.
static void computeTargets(float dt, uint32_t now) {   // caller holds mux
  if (!S.gaitOn) S.unstickLeg = -1;
  if (S.unstickLeg >= 0 && now - S.unstickStartMs >= UNSTICK_MS) S.unstickLeg = -1;   // auto-resume
  if (S.gaitOn && S.unstickLeg < 0) S.phase = fmodf(S.phase + S.gaitFreq * dt, 1.0f);
  for (int i = 0; i < 4; i++) {
    if (!S.attached[i] || !S.gaitOn) continue;   // motor off, target frozen
    if (S.unstickLeg >= 0) {
      if (i == S.unstickLeg) {
        float t = (now - S.unstickStartMs) / 1000.0f;
        S.target[i] = S.unstickCenter + UNSTICK_AMP_DEG * sinf(TWO_PI_F * UNSTICK_FREQ_HZ * t);
      }                                            // other legs hold still while it wiggles
    } else {
      // Turning: longer strides on one side. Left legs (FL, RL) grow when turning right.
      float side = (i == 0 || i == 2) ? 1.0f : -1.0f;
      float amp = S.gaitAmp * (1.0f + 0.5f * S.gaitTurn * side);
      S.target[i] = amp * sinf(TWO_PI_F * (S.phase + PHASE_OFFSET[i]));
    }
  }
}

static void controlTask(void*) {
  const float dt = 1.0f / CONTROL_HZ;
  TickType_t last = xTaskGetTickCount();
#if !SIM_SENSORS
  float prevErr[4] = {0, 0, 0, 0};
#endif
  for (;;) {
#if !SIM_SENSORS
    float enc[4];
    readEncoders(enc);
#endif
    float tgt[4];
    bool att[4], on;
    uint32_t now = millis();
    portENTER_CRITICAL(&mux);
    computeTargets(dt, now);
    for (int i = 0; i < 4; i++) {
      tgt[i] = S.target[i];
      att[i] = S.attached[i];
#if !SIM_SENSORS
      S.enc[i] = enc[i];
#endif
    }
    on = S.gaitOn;
    portEXIT_CRITICAL(&mux);
#if !SIM_SENSORS
    for (int i = 0; i < 4; i++) {
      if (!on || !att[i]) {
        setMotor(i, 0);
        prevErr[i] = 0;
        continue;
      }
      float err = tgt[i] - enc[i];
      float dErr = (err - prevErr[i]) / dt;
      prevErr[i] = err;
      setMotor(i, fmaxf(-MAX_PWM, fminf(MAX_PWM, KP * err + KD * dErr)));
    }
#else
    (void)tgt; (void)att; (void)on;
#endif
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
  }
}

// ====================================================================== commands
static int legIndex(const char* name) {
  if (!name) return -1;
  for (int i = 0; i < 4; i++) if (strcmp(name, LEG_NAMES[i]) == 0) return i;
  return -1;
}

static void stopMotors(const char* why) {
  portENTER_CRITICAL(&mux);
  bool wasOn = S.gaitOn;
  S.gaitOn = false;
  S.unstickLeg = -1;
  portEXIT_CRITICAL(&mux);
  if (wasOn) Serial.printf("Motors off: %s\n", why);
}

static void handlePacket(const char* data, int len, IPAddress ip, uint16_t port) {
  JsonDocument doc;
  if (deserializeJson(doc, data, len)) return;   // not JSON: ignore
  bool newPc = !havePc || !(ip == pcIP) || port != pcPort;
  pcIP = ip;
  pcPort = port;
  havePc = true;
  lastPcMs = millis();
  if (newPc) Serial.printf("PC at %s:%u\n", ip.toString().c_str(), port);

  const char* cmd = doc["cmd"] | "";
  int leg = legIndex(doc["leg"].as<const char*>());   // nullptr if missing or null

  if (strcmp(cmd, "hello") == 0) {
    char reply[96];
    int n = snprintf(reply, sizeof(reply), "{\"hello\":\"%s\",\"sim\":%d,\"ap\":%d}", HOSTNAME, SIM_SENSORS, apMode ? 1 : 0);
    sendTo(ip, port, reply, n);
  } else if (strcmp(cmd, "hb") == 0) {
    // heartbeat: lastPcMs already updated
  } else if (strcmp(cmd, "gait") == 0) {
    int on = doc["on"] | 1;
    portENTER_CRITICAL(&mux);
    S.gaitFreq = doc["freq"] | S.gaitFreq;
    S.gaitAmp = doc["amp"] | S.gaitAmp;
    S.gaitTurn = fmaxf(-1.0f, fminf(1.0f, doc["turn"] | 0.0f));
    S.gaitOn = on != 0;
    if (!S.gaitOn) S.unstickLeg = -1;
    portEXIT_CRITICAL(&mux);
    Serial.printf("Gait %s (%.2f Hz, %.0f deg, turn %.2f)\n", on ? "on" : "off", S.gaitFreq, S.gaitAmp, S.gaitTurn);
  } else if (strcmp(cmd, "stop") == 0) {
    stopMotors("stop command");
  } else if (strcmp(cmd, "unstick") == 0 && leg >= 0) {
    portENTER_CRITICAL(&mux);
    if (S.gaitOn && S.attached[leg]) {
      S.unstickLeg = leg;
      S.unstickStartMs = millis();
      S.unstickCenter = S.enc[leg];
    }
    portEXIT_CRITICAL(&mux);
    Serial.printf("Unstick %s\n", LEG_NAMES[leg]);
#if SIM_SENSORS
    sim.freeAfterUnstick = urand() < SIM_UNSTICK_FREE_CHANCE;
#endif
  } else if (strcmp(cmd, "resume") == 0) {
    portENTER_CRITICAL(&mux);
    S.unstickLeg = -1;
    portEXIT_CRITICAL(&mux);
  } else if (strcmp(cmd, "detach") == 0 && leg >= 0) {
    portENTER_CRITICAL(&mux);
    S.attached[leg] = false;
    if (S.unstickLeg == leg) S.unstickLeg = -1;
    portEXIT_CRITICAL(&mux);
    setServo(leg, SERVO_RELEASE_DEG);
    Serial.printf("Detach %s\n", LEG_NAMES[leg]);
  } else if (strcmp(cmd, "attach") == 0 && leg >= 0) {
    setServo(leg, SERVO_LOCK_DEG);
    portENTER_CRITICAL(&mux);
    S.attached[leg] = true;
    portEXIT_CRITICAL(&mux);
    Serial.printf("Attach %s\n", LEG_NAMES[leg]);
  } else if (strcmp(cmd, "zero") == 0) {
#if SIM_SENSORS
    for (int i = 0; i < 4; i++) sim.enc[i] = 0;
#else
    zeroEncoders();
#endif
  }
#if SIM_SENSORS
  else if (strcmp(cmd, "sim_jam") == 0) {
    sim.jam = leg;   // -1 frees it
    sim.strength = doc["s"] | 1.0f;
    Serial.printf("Sim: %s\n", leg >= 0 ? LEG_NAMES[leg] : "no leg jammed");
  }
#endif
}

// ====================================================================== telemetry
static int buildTelemetry(char* buf, size_t size) {
  float enc[4], tgt[4], cur[4], ph;
  int att[4], gait;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) {
    enc[i] = S.enc[i];
    tgt[i] = S.target[i];
    cur[i] = S.cur[i];
    att[i] = S.attached[i] ? 1 : 0;
  }
  ph = S.phase;
  gait = S.gaitOn ? 1 : 0;
  portEXIT_CRITICAL(&mux);
#if SIM_SENSORS
  int jam = sim.jam;
#else
  int jam = -1;
#endif
  return snprintf(buf, size,
                  "{\"t\":%lu,\"imu\":[%.3f,%.3f,%.3f,%.2f,%.2f,%.2f],"
                  "\"tof\":[%.1f,%.1f,%.1f,%.1f],\"cur\":[%.1f,%.1f,%.1f,%.1f],"
                  "\"enc\":[%.2f,%.2f,%.2f,%.2f],\"cmd\":[%.2f,%.2f,%.2f,%.2f],"
                  "\"ph\":%.3f,\"gait\":%d,\"att\":[%d,%d,%d,%d],\"sim\":%d,\"jam\":%d}",
                  (unsigned long)millis(), imu[0], imu[1], imu[2], imu[3], imu[4], imu[5],
                  tofMm[0], tofMm[1], tofMm[2], tofMm[3], cur[0], cur[1], cur[2], cur[3],
                  enc[0], enc[1], enc[2], enc[3], tgt[0], tgt[1], tgt[2], tgt[3],
                  ph, gait, att[0], att[1], att[2], att[3], SIM_SENSORS, jam);
}

// ====================================================================== setup / loop
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\nRobot dog firmware (%s)\n", SIM_SENSORS ? "SIMULATED sensors" : "real hardware");
#if !SIM_SENSORS
  motorsBegin();
  sensorsBegin();   // starts I2C; must come before the PCA9685
#endif
  servosBegin();
#if !SIM_SENSORS && HAS_TOF
  tofBegin();       // may need the PCA9685 for XSHUT, so it runs after servosBegin
  for (int i = 0; i < 4; i++) Serial.printf("ToF %s: %s\n", LEG_NAMES[i], tofOk[i] ? "ok" : "NOT FOUND");
#endif
  startWifi();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  static char rx[512];
  static char tx[512];
  static uint32_t nextTelemetryUs = micros();
  const uint32_t periodUs = 1000000UL / TELEMETRY_HZ;

  // 1. Commands from the PC.
  int size;
  while ((size = udp.parsePacket()) > 0) {
    int n = udp.read(rx, sizeof(rx) - 1);
    if (n <= 0) continue;
    rx[n] = 0;
    handlePacket(rx, n, udp.remoteIP(), udp.remotePort());
  }

  // 2. Safety: PC went quiet -> stop.
  if (havePc && millis() - lastPcMs > HEARTBEAT_TIMEOUT_MS) stopMotors("no heartbeat from PC");

  // 3. Sensors + telemetry at TELEMETRY_HZ.
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextTelemetryUs) >= 0) {
    nextTelemetryUs += periodUs;
    if ((int32_t)(nowUs - nextTelemetryUs) > (int32_t)periodUs) nextTelemetryUs = nowUs + periodUs;   // fell behind
#if SIM_SENSORS
    simStep(1.0f / TELEMETRY_HZ);
#else
    readBodySensors();
#endif
    if (havePc && millis() - lastPcMs < PC_IDLE_MS) {
      int n = buildTelemetry(tx, sizeof(tx));
      if (n > 0 && n < (int)sizeof(tx)) sendTo(pcIP, pcPort, tx, n);
    }
  }
  delay(1);   // let Wi-Fi and the idle task run
}
