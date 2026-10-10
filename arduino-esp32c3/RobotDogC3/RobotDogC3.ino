// Robot dog for the ESP32-C3, written for the Arduino IDE.
//
// It talks to the laptop exactly like esp32-firmware/ does, so everything in
// stuck-leg/ (logger, train, live, dashboard, Qwen commander) works unchanged.
// The ML runs on the laptop; this board streams sensor data and drives the motors.
//
// What is different from esp32-firmware/:
//   - ESP32-C3 (one RISC-V core, 6 PWM channels): no background task, everything runs in loop(),
//     and each motor uses one PWM pin + one direction pin.
//   - No encoders: the N20s run open loop. The "enc" values sent to the laptop equal the
//     commanded angles, and "enc_ok":0 tells the laptop there are no real encoders.
//   - SIM_SENSORS 1 + MOTORS_ENABLED 1 = the real motors walk while the sensor numbers
//     come from the built-in simulator, so the ML and dashboard run during a real walk.
//
// Settings: config.h (the other tab in the Arduino IDE).
//
// Protocol (JSON over UDP, port UDP_PORT):
//   laptop -> robot  {"cmd":"hello"}                 discovery (broadcast); robot replies {"hello":"robotdog",...}
//                    {"cmd":"hb"}                    heartbeat, every 100 ms; silence > 300 ms = motors off
//                    {"cmd":"gait","on":1,"freq":1.5,"amp":25,"turn":0}   start/stop walking; turn -1..1 (+ = right)
//                    {"cmd":"stop"}                  motors off
//                    {"cmd":"unstick","leg":"FL"}    rock that leg back and forth for UNSTICK_MS, then resume
//                    {"cmd":"resume"}                end an unstick early
//                    {"cmd":"detach","leg":"FL"}     that leg's motor off. Refused unless the legs off afterwards
//                                                    are one leg or a diagonal pair (FL+RR, FR+RL)
//                    {"cmd":"attach","leg":"FL"}     leg rejoins the gait
//                    {"cmd":"zero"}                  reset the simulated leg angles
//                    {"cmd":"sim_jam","leg":"FL","s":1.0}  SIM_SENSORS only: jam one more leg (leg null = free all)
//   robot -> laptop  {"t":ms,"imu":[ax,ay,az,gx,gy,gz],"tof":[4],"cur":[4],"enc":[4],"cmd":[4],"ph":phase,
//                     "gait":0/1,"att":[4],"sim":0/1,"jam":-1..3,"jams":[4],"enc_ok":0/1}   at TELEMETRY_HZ

#include <Arduino.h>
#include <esp_arduino_version.h>   // ESP_ARDUINO_VERSION_MAJOR (2.x and 3.x cores differ in the PWM API)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>   // Library Manager: "ArduinoJson" by Benoit Blanchon, version 7
#include <math.h>
#include <string.h>
#include "config.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "This sketch is for the ESP32-C3. In Tools > Board, pick ESP32C3 Dev Module (or your C3 board)."
#endif

#if !SIM_SENSORS
#include <Wire.h>
#if HAS_IMU
#include <Adafruit_MPU6050.h>   // Library Manager: "Adafruit MPU6050"
#endif
#if HAS_CURRENT
#include <Adafruit_INA219.h>    // Library Manager: "Adafruit INA219"
#endif
#if HAS_TOF
#include <VL53L0X.h>            // Library Manager: "VL53L0X" by Pololu
#endif
#endif

// Encoders are only ever simulated; the real robot has none.
#define REPORT_ENCODERS (SIM_SENSORS && SIM_HAS_ENCODERS)

static const char* LEG_NAMES[4] = {"FL", "FR", "RL", "RR"};
static const float TWO_PI_F = 6.28318530718f;

// ====================================================================== state
struct State {
  bool gaitOn = false;
  float gaitFreq = GAIT_FREQ_HZ;
  float gaitAmp = GAIT_AMP_DEG;
  float gaitTurn = 0.0f;            // -1..1; + = turn right
  float phase = 0.0f;               // 0..1 through one trot cycle
  bool attached[4] = {true, true, true, true};
  int unstickLeg = -1;              // -1 = not unsticking
  uint32_t unstickStartMs = 0;
  float unstickCenter = 0.0f;
  float target[4] = {0, 0, 0, 0};   // commanded leg angle, deg (the trot pattern)
  float enc[4] = {0, 0, 0, 0};      // reported leg angle, deg (= target unless simulating encoders)
  float cur[4] = {0, 0, 0, 0};      // motor current, mA
  int duty[4] = {0, 0, 0, 0};       // what each motor is actually being driven at, -MAX_DUTY..MAX_DUTY
};
static State S;

static float imu[6] = {0, 0, 9.81f, 0, 0, 0};   // m/s^2, deg/s
static float tofMm[4] = {0, 0, 0, 0};

// ====================================================================== Wi-Fi
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
#if WIFI_LOW_TX_POWER
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
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
#if WIFI_LOW_TX_POWER
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
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

// ====================================================================== motors
// DRV8833 truth table: IN1/IN2 = 0/0 coast, 1/0 forward, 0/1 reverse, 1/1 brake.
// Forward: DIR low, PWM = duty            -> alternates forward / coast.
// Reverse: DIR high, PWM = full - duty    -> alternates reverse / brake, so the motor
//          still gets `duty` worth of reverse drive with only one PWM pin.
static const int PWM_FULL = (1 << MOTOR_PWM_BITS) - 1;

static void pwmWrite(int i, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM[i], duty);   // Arduino-ESP32 3.x: by pin
#else
  ledcWrite(i, duty);              // Arduino-ESP32 2.x: by channel (channel i = motor i)
#endif
}

static void motorsBegin() {
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_DIR_PIN[i], OUTPUT);
    digitalWrite(MOTOR_DIR_PIN[i], LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PWM[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(MOTOR_PWM[i], i);
#endif
    pwmWrite(i, 0);
  }
}

// duty in [-MAX_DUTY, MAX_DUTY]; + = forward, 0 = coast.
static void setMotor(int i, float duty) {
  int d = (int)lroundf(duty) * MOTOR_FLIP[i];
  int mag = abs(d);
  if (mag > MAX_DUTY) mag = MAX_DUTY;
  if (mag < MIN_DUTY) mag = 0;                    // too weak to turn an N20: don't just heat it
  S.duty[i] = (d < 0 ? -mag : mag) * MOTOR_FLIP[i];
#if MOTORS_ENABLED
  if (mag == 0) {
    digitalWrite(MOTOR_DIR_PIN[i], LOW);
    pwmWrite(i, 0);
  } else if (d > 0) {
    digitalWrite(MOTOR_DIR_PIN[i], LOW);
    pwmWrite(i, mag);
  } else {
    digitalWrite(MOTOR_DIR_PIN[i], HIGH);
    pwmWrite(i, PWM_FULL - mag);
  }
#endif
}

// ====================================================================== real sensors
#if !SIM_SENSORS
#if HAS_IMU
static Adafruit_MPU6050 mpu;
static bool mpuOk = false;
#endif
#if HAS_CURRENT
static Adafruit_INA219 ina[4] = {Adafruit_INA219(INA219_ADDR[0]), Adafruit_INA219(INA219_ADDR[1]),
                                 Adafruit_INA219(INA219_ADDR[2]), Adafruit_INA219(INA219_ADDR[3])};
static bool inaOk[4] = {false, false, false, false};
#endif
#if HAS_TOF
static VL53L0X tof;
static bool tofOk = false;
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
  Serial.printf("IMU: %s\n", mpuOk ? "ok" : "NOT FOUND (check SDA/SCL and 3.3V)");
#endif
#if HAS_CURRENT
  for (int i = 0; i < 4; i++) {
    inaOk[i] = ina[i].begin();
    Serial.printf("Current sensor %s (0x%02X): %s\n", LEG_NAMES[i], INA219_ADDR[i], inaOk[i] ? "ok" : "NOT FOUND");
  }
#endif
#if HAS_TOF
  tof.setTimeout(30);
  tofOk = tof.init();
  if (tofOk) {
    tof.setMeasurementTimingBudget(20000);
    tof.startContinuous();
  }
  Serial.printf("ToF: %s\n", tofOk ? "ok" : "NOT FOUND");
#endif
}

static void readSensors() {
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
#if HAS_CURRENT
  for (int i = 0; i < 4; i++) if (inaOk[i]) S.cur[i] = fabsf(ina[i].getCurrent_mA());
#endif
#if HAS_TOF
  // Non-blocking: only read when a new measurement is ready.
  if (tofOk && (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0) {
    uint16_t mm = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (mm < 8190) for (int i = 0; i < 4; i++) tofMm[i] = mm;   // 8190/8191 = out of range
  }
#endif
  for (int i = 0; i < 4; i++) S.enc[i] = S.target[i];   // no encoders: report the command
}
#endif  // !SIM_SENSORS

// ====================================================================== simulated sensors
// A C++ port of stuck-leg/fake_robot.py, stepped at TELEMETRY_HZ, so a model trained on
// fake data recognises jams made from the dashboard here too. The real motors (if enabled)
// keep walking; only the numbers sent to the laptop are made up.
#if SIM_SENSORS
struct Sim {
  float enc[4] = {0, 0, 0, 0};
  float lastCmd[4] = {0, 0, 0, 0};
  float tilt[2] = {0, 0};        // roll, pitch (rad)
  float terrain[2] = {0, 0};
  float bump = 0, turn = 0, turnTarget = 0;
  bool jam[4] = {false, false, false, false};   // jammed legs (any number)
  float strength[4] = {1, 1, 1, 1};             // 1 = hard jam, <1 = partial snag
  int unstickSeen = -1;          // leg being rocked at the last step
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
  const bool on = S.gaitOn;

  int bumpLeg = -1;
  if (urand() < 0.02f * rough) { sim.bump = 1.0f; bumpLeg = (int)(urand() * 4) & 3; }
  sim.bump *= 0.85f;
  for (int k = 0; k < 2; k++) {
    sim.terrain[k] = fmaxf(-0.05f, fminf(0.05f, sim.terrain[k] + gauss(0.004f * rough)));
  }
  if (urand() < 0.01f) { const float t[7] = {0, 0, 0, -20, 20, -40, 40}; sim.turnTarget = t[(int)(urand() * 7) % 7]; }
  sim.turn += (sim.turnTarget - sim.turn) * fminf(1.0f, 3 * dt);

  float sag[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    float c = S.target[i];
    float dc = (c - sim.lastCmd[i]) / dt;   // deg/s
    sim.lastCmd[i] = c;
    float current;
    if (!S.attached[i] || !on) {            // motor off: leg stays put, no load
      current = fabsf(gauss(5.0f));
    } else if (sim.jam[i]) {
      float follow = 1.0f - sim.strength[i];
      sim.enc[i] += (c - sim.enc[i]) * fminf(1.0f, 8 * dt) * follow + gauss(0.4f);
      current = (350 + 250 * sim.strength[i]) + gauss(40);
      sag[i] = -7 * sim.strength[i];
    } else {
      sim.enc[i] += (c - sim.enc[i]) * fminf(1.0f, 15 * dt) + gauss(0.6f * rough);
      float stance = (c < 0) ? 1.0f : 0.0f;  // foot on the ground half the cycle
      current = 120 + 1.2f * fabsf(dc) + 90 * stance + gauss(25 * rough);
    }
    if (S.attached[i] && on && (i == bumpLeg || (sim.bump > 0.3f && urand() < 0.15f))) current += 220 * sim.bump;
    S.cur[i] = fmaxf(0.0f, current);
    S.enc[i] = REPORT_ENCODERS ? sim.enc[i] : c;
  }

  float tRoll = 0, tPitch = 0, yaw = 0;
  for (int i = 0; i < 4 && on; i++) {      // each jammed corner pulls the body toward it
    if (!sim.jam[i]) continue;
    float s = sim.strength[i];
    tPitch += -0.06f * CORNER_X[i] * s;
    tRoll += 0.06f * CORNER_Y[i] * s;
    yaw += 15 * CORNER_Y[i] * s;
  }
  sim.tilt[0] += (tRoll - sim.tilt[0]) * fminf(1.0f, 4 * dt);
  sim.tilt[1] += (tPitch - sim.tilt[1]) * fminf(1.0f, 4 * dt);
  float roll = sim.tilt[0] + sim.terrain[0], pitch = sim.tilt[1] + sim.terrain[1];

  if (sim.unstickSeen >= 0 && S.unstickLeg < 0) {        // an unstick just finished: did it come loose?
    if (sim.jam[sim.unstickSeen] && sim.freeAfterUnstick) {
      Serial.printf("Sim: unstick freed %s\n", LEG_NAMES[sim.unstickSeen]);
      sim.jam[sim.unstickSeen] = false;
    }
    sim.freeAfterUnstick = false;
  }
  sim.unstickSeen = S.unstickLeg;

  float ph = S.phase;
  float bounce = on ? 0.6f * sinf(2 * TWO_PI_F * ph) : 0.0f;
  float n = 0.25f * rough;
  imu[0] = -G * sinf(pitch) + gauss(n) + 1.5f * sim.bump * gauss(1);
  imu[1] = G * sinf(roll) + gauss(n);
  imu[2] = G * cosf(roll) * cosf(pitch) + bounce + gauss(n);
  imu[3] = gauss(3 * rough) + 40 * (tRoll - sim.tilt[0]);
  imu[4] = gauss(3 * rough) + 40 * (tPitch - sim.tilt[1]);
  imu[5] = yaw + (on ? sim.turn - 30 * S.gaitTurn : 0) + gauss(4 * rough);   // turning right = negative yaw rate

  for (int i = 0; i < 4; i++) {
    float h = 80.0f + 150 * (CORNER_X[i] * sinf(pitch) - CORNER_Y[i] * sinf(roll));
    h += 3 * sinf(TWO_PI_F * (ph + PHASE_OFFSET[i])) + sag[i] + gauss(2.5f * rough);
    if (urand() < 0.01f) h += (urand() < 0.5f ? -1 : 1) * (15 + 25 * urand());   // ToF glitch
    tofMm[i] = h;
  }
}
#endif  // SIM_SENSORS

// ====================================================================== gait + motors
// The trot pattern is computed as leg angles (S.target) even in WHEG style: the laptop,
// the simulator and the dashboard's leg animation all use it. The motors are then driven
// open loop from it.
static void controlStep(float dt, uint32_t now) {
  if (!S.gaitOn) S.unstickLeg = -1;
  if (S.unstickLeg >= 0 && now - S.unstickStartMs >= UNSTICK_MS) S.unstickLeg = -1;   // auto-resume
  if (S.gaitOn && S.unstickLeg < 0) S.phase = fmodf(S.phase + S.gaitFreq * dt, 1.0f);

  float prev[4];                                   // last targets, for SWING style's speed
  for (int i = 0; i < 4; i++) prev[i] = S.target[i];
  (void)prev;
  for (int i = 0; i < 4; i++) {
    if (!S.attached[i] || !S.gaitOn) continue;   // motor off, target frozen
    if (S.unstickLeg >= 0) {
      if (i == S.unstickLeg) {
        float t = (now - S.unstickStartMs) / 1000.0f;
        S.target[i] = S.unstickCenter + UNSTICK_AMP_DEG * sinf(TWO_PI_F * UNSTICK_FREQ_HZ * t);
      }                                            // other legs hold still while it rocks
    } else {
      // Turning: longer strides on one side. Left legs (FL, RL) grow when turning right.
      float side = (i == 0 || i == 2) ? 1.0f : -1.0f;
      float amp = S.gaitAmp * (1.0f + 0.5f * S.gaitTurn * side);
      S.target[i] = amp * sinf(TWO_PI_F * (S.phase + PHASE_OFFSET[i]));
    }
  }

  for (int i = 0; i < 4; i++) {
    if (!S.gaitOn || !S.attached[i]) { setMotor(i, 0); continue; }
    float duty;
    if (S.unstickLeg >= 0) {
      if (i != S.unstickLeg) { setMotor(i, 0); continue; }
#if LEG_STYLE == LEG_STYLE_WHEG
      float t = (now - S.unstickStartMs) / 1000.0f;
      duty = UNSTICK_DUTY * sinf(TWO_PI_F * UNSTICK_FREQ_HZ * t);   // rock: forward, back, forward...
#else
      duty = SWING_DUTY_PER_DEG_S * (S.target[i] - prev[i]) / dt;
#endif
    } else {
#if LEG_STYLE == LEG_STYLE_WHEG
      float side = (i == 0 || i == 2) ? 1.0f : -1.0f;
      duty = WHEG_DUTY * (S.gaitFreq / GAIT_FREQ_HZ) * (1.0f + 0.5f * S.gaitTurn * side);
#else
      duty = SWING_DUTY_PER_DEG_S * (S.target[i] - prev[i]) / dt;   // feed-forward: speed of the pattern
#endif
    }
    setMotor(i, duty);
  }
}

// ====================================================================== commands
static int legIndex(const char* name) {
  if (!name) return -1;
  for (int i = 0; i < 4; i++) if (strcmp(name, LEG_NAMES[i]) == 0) return i;
  return -1;
}

static void stopMotors(const char* why) {
  bool wasOn = S.gaitOn;
  S.gaitOn = false;
  S.unstickLeg = -1;
  for (int i = 0; i < 4; i++) setMotor(i, 0);
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
  if (newPc) Serial.printf("Laptop at %s:%u\n", ip.toString().c_str(), port);

  const char* cmd = doc["cmd"] | "";
  int leg = legIndex(doc["leg"].as<const char*>());   // -1 if missing or null

  if (strcmp(cmd, "hello") == 0) {
    char reply[128];
    int n = snprintf(reply, sizeof(reply), "{\"hello\":\"%s\",\"sim\":%d,\"ap\":%d,\"enc\":%d,\"board\":\"esp32c3\"}",
                     HOSTNAME, SIM_SENSORS, apMode ? 1 : 0, REPORT_ENCODERS);
    sendTo(ip, port, reply, n);
  } else if (strcmp(cmd, "hb") == 0) {
    // heartbeat: lastPcMs already updated
  } else if (strcmp(cmd, "gait") == 0) {
    int on = doc["on"] | 1;
    S.gaitFreq = doc["freq"] | S.gaitFreq;
    S.gaitAmp = doc["amp"] | S.gaitAmp;
    S.gaitTurn = fmaxf(-1.0f, fminf(1.0f, doc["turn"] | 0.0f));
    S.gaitOn = on != 0;
    if (!S.gaitOn) stopMotors("gait off");
    Serial.printf("Gait %s (%.2f Hz, %.0f deg, turn %.2f)\n", on ? "on" : "off", S.gaitFreq, S.gaitAmp, S.gaitTurn);
  } else if (strcmp(cmd, "stop") == 0) {
    stopMotors("stop command");
  } else if (strcmp(cmd, "unstick") == 0 && leg >= 0) {
    if (S.gaitOn && S.attached[leg]) {
      S.unstickLeg = leg;
      S.unstickStartMs = millis();
      S.unstickCenter = S.target[leg];
    }
    Serial.printf("Unstick %s\n", LEG_NAMES[leg]);
#if SIM_SENSORS
    sim.freeAfterUnstick = urand() < SIM_UNSTICK_FREE_CHANCE;
#endif
  } else if (strcmp(cmd, "resume") == 0) {
    S.unstickLeg = -1;
  } else if (strcmp(cmd, "detach") == 0 && leg >= 0) {
    // Safety rule: the legs off afterwards must be just this one, or this one plus its
    // diagonal partner (FL+RR, FR+RL). Otherwise the robot couldn't balance.
    int partner = 3 - leg;
    for (int i = 0; i < 4; i++) {
      if (!S.attached[i] && i != leg && i != partner) {
        Serial.printf("Refused detach %s: only diagonal pairs (FL+RR, FR+RL) can be off together\n", LEG_NAMES[leg]);
        return;
      }
    }
    S.attached[leg] = false;
    if (S.unstickLeg == leg) S.unstickLeg = -1;
    setMotor(leg, 0);
#if SIM_SENSORS
    sim.jam[leg] = false;                 // a dropped leg can't stay stuck
#endif
    Serial.printf("Detach %s (motor off; the magnets let the leg come away)\n", LEG_NAMES[leg]);
  } else if (strcmp(cmd, "attach") == 0 && leg >= 0) {
    S.attached[leg] = true;
    Serial.printf("Attach %s\n", LEG_NAMES[leg]);
  } else if (strcmp(cmd, "zero") == 0) {
    for (int i = 0; i < 4; i++) S.target[i] = 0;
#if SIM_SENSORS
    for (int i = 0; i < 4; i++) sim.enc[i] = 0;
#endif
  }
#if SIM_SENSORS
  else if (strcmp(cmd, "sim_jam") == 0) {
    if (leg < 0) {                        // no leg: free them all
      for (int i = 0; i < 4; i++) sim.jam[i] = false;
      Serial.println("Sim: all legs freed");
    } else if (S.attached[leg]) {         // jam one more leg
      sim.jam[leg] = true;
      sim.strength[leg] = doc["s"] | 1.0f;
      Serial.printf("Sim: jammed %s\n", LEG_NAMES[leg]);
    }
  }
#endif
}

// ====================================================================== telemetry
static int buildTelemetry(char* buf, size_t size) {
  int jams[4] = {0, 0, 0, 0}, jam = -1;
#if SIM_SENSORS
  for (int i = 3; i >= 0; i--) if (sim.jam[i]) { jams[i] = 1; jam = i; }   // jam = first jammed leg
#endif
  return snprintf(buf, size,
                  "{\"t\":%lu,\"imu\":[%.3f,%.3f,%.3f,%.2f,%.2f,%.2f],"
                  "\"tof\":[%.1f,%.1f,%.1f,%.1f],\"cur\":[%.1f,%.1f,%.1f,%.1f],"
                  "\"enc\":[%.2f,%.2f,%.2f,%.2f],\"cmd\":[%.2f,%.2f,%.2f,%.2f],"
                  "\"ph\":%.3f,\"gait\":%d,\"att\":[%d,%d,%d,%d],\"sim\":%d,\"jam\":%d,\"jams\":[%d,%d,%d,%d],"
                  "\"enc_ok\":%d}",
                  (unsigned long)millis(), imu[0], imu[1], imu[2], imu[3], imu[4], imu[5],
                  tofMm[0], tofMm[1], tofMm[2], tofMm[3], S.cur[0], S.cur[1], S.cur[2], S.cur[3],
                  S.enc[0], S.enc[1], S.enc[2], S.enc[3], S.target[0], S.target[1], S.target[2], S.target[3],
                  S.phase, S.gaitOn ? 1 : 0, S.attached[0], S.attached[1], S.attached[2], S.attached[3],
                  SIM_SENSORS, jam, jams[0], jams[1], jams[2], jams[3], REPORT_ENCODERS);
}

// ====================================================================== setup / loop
void setup() {
  Serial.begin(115200);
  delay(1500);   // give USB serial time to connect so the first lines aren't lost
  Serial.printf("\nRobot dog (ESP32-C3): %s sensors, motors %s, legs: %s\n",
                SIM_SENSORS ? "SIMULATED" : "real", MOTORS_ENABLED ? "ON" : "off",
                LEG_STYLE == LEG_STYLE_WHEG ? "spinning whegs" : "swinging");
  motorsBegin();
#if !SIM_SENSORS
  sensorsBegin();
#endif
  startWifi();
}

void loop() {
  static char rx[512];
  static char tx[512];
  static uint32_t nextControlUs = micros();
  static uint32_t nextTelemetryUs = micros();
  const uint32_t controlUs = 1000000UL / CONTROL_HZ;
  const uint32_t telemetryUs = 1000000UL / TELEMETRY_HZ;

  // 1. Commands from the laptop.
  int size;
  while ((size = udp.parsePacket()) > 0) {
    int n = udp.read(rx, sizeof(rx) - 1);
    if (n <= 0) continue;
    rx[n] = 0;
    handlePacket(rx, n, udp.remoteIP(), udp.remotePort());
  }

  // 2. Safety: laptop went quiet -> stop.
  if (havePc && millis() - lastPcMs > HEARTBEAT_TIMEOUT_MS) stopMotors("no heartbeat from laptop");

  // 3. Gait + motors at CONTROL_HZ.
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextControlUs) >= 0) {
    nextControlUs += controlUs;
    if ((int32_t)(nowUs - nextControlUs) > (int32_t)controlUs) nextControlUs = nowUs + controlUs;   // fell behind
    controlStep(1.0f / CONTROL_HZ, millis());
  }

  // 4. Sensors + telemetry at TELEMETRY_HZ.
  nowUs = micros();
  if ((int32_t)(nowUs - nextTelemetryUs) >= 0) {
    nextTelemetryUs += telemetryUs;
    if ((int32_t)(nowUs - nextTelemetryUs) > (int32_t)telemetryUs) nextTelemetryUs = nowUs + telemetryUs;
#if SIM_SENSORS
    simStep(1.0f / TELEMETRY_HZ);
#else
    readSensors();
#endif
    if (havePc && millis() - lastPcMs < PC_IDLE_MS) {
      int n = buildTelemetry(tx, sizeof(tx));
      if (n > 0 && n < (int)sizeof(tx)) sendTo(pcIP, pcPort, tx, n);
    }
  }
  delay(1);   // let Wi-Fi run (the C3 has one core, shared with the Wi-Fi stack)
}
