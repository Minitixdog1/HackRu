// Everything you are likely to change lives in this file.
// Leg order is FL, FR, RL, RR in every array (front-left, front-right, rear-left, rear-right).
#pragma once
#include <stdint.h>

// ============================================================ Wi-Fi
// The robot first tries to join this network (a phone hotspot works best at a
// hackathon; campus Wi-Fi usually blocks laptop-to-device traffic).
#define WIFI_SSID "your-hotspot-name"
#define WIFI_PASS "your-hotspot-password"
#define WIFI_CONNECT_TIMEOUT_MS 15000

// If it can't join within the timeout, it makes its own network instead.
// Join "robotdog" from your laptop (password below) and everything still works.
#define AP_SSID "robotdog"
#define AP_PASS "robotdog123"          // at least 8 characters
#define FORCE_AP_MODE 0                // 1 = skip WIFI_SSID and always make its own network

#define HOSTNAME "robotdog"            // also reachable as robotdog.local
#define UDP_PORT 4210                  // must match the laptop code (stuck-leg/robot_link.py)

// Many cheap "ESP32-C3 Super Mini" boards can't join Wi-Fi at full transmit power
// (a known antenna problem). If it never connects, set this to 1.
#define WIFI_LOW_TX_POWER 0

// ============================================================ Timing and safety
#define TELEMETRY_HZ 50                // sensor messages per second to the laptop
#define CONTROL_HZ 200                 // motor update rate
#define HEARTBEAT_TIMEOUT_MS 300       // no message from the laptop this long -> motors off
#define PC_IDLE_MS 2000                // stop streaming if the laptop has been quiet this long

// ============================================================ Modes
// SIM_SENSORS 1: the sensor numbers sent to the laptop come from a built-in
//   simulator (the same model as stuck-leg/fake_robot.py). The laptop can "jam"
//   a leg from the dashboard and the ML sees it. Nothing needs to be wired.
// SIM_SENSORS 0: read the real sensors listed under "Real sensors" below.
#define SIM_SENSORS 1

// MOTORS_ENABLED 1: the real N20 motors are driven, even while SIM_SENSORS is 1.
//   That is the "simulation runs while it walks" mode: the dog really walks,
//   and the ML and dashboard run on simulated sensor data at the same time.
// MOTORS_ENABLED 0: motors never move (bare board on a desk).
#define MOTORS_ENABLED 1

// Our robot has no encoders. With 0, the simulator also pretends there are none
// (it reports each leg's angle as equal to its command), so a model trained on
// this data learns from current, IMU and ToF only, like it must on the real robot.
// Set to 1 only to use an old model that was trained with fake encoder data.
#define SIM_HAS_ENCODERS 0

// ============================================================ How the legs move
// No encoders means the motors run open loop: we set a speed, nothing measures the angle.
// LEG_STYLE_WHEG:  every leg spins continuously (rotating spoke, part 05 in Onshapefile).
// LEG_STYLE_SWING: every leg swings back and forth following the trot pattern. Without
//                  encoders the swing slowly drifts, so WHEG is the better default.
#define LEG_STYLE_WHEG 0
#define LEG_STYLE_SWING 1
#define LEG_STYLE LEG_STYLE_WHEG

// ============================================================ Pins (ESP32-C3)
// Safe pins on most C3 boards (Super Mini, DevKitM-1, generic C3 modules):
//   free:      0, 1, 3, 4, 5, 6, 7, 10
//   I2C:       8 (SDA), 9 (SCL). These are boot pins, but I2C lines idle HIGH, which is what boot needs.
//   avoid:     2 (boot pin), 11-17 (flash), 18/19 (USB on Super Mini), 20/21 (USB-serial chip on DevKitM-1)
// Seeed XIAO ESP32-C3: only 11 pins come out, labelled D0-D10 (D4 = GPIO 6 = SDA, D5 = GPIO 7 = SCL).
// Use GPIO numbers here, not D numbers, and check your board's pinout picture.
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_HZ 400000

// Motor driver: DRV8833 (two boards, two motors each), or anything with IN1/IN2 inputs.
// The C3 has only 6 PWM channels, so each motor uses ONE PWM pin + ONE plain direction pin:
//   motor wire IN1 <- MOTOR_PWM pin, motor wire IN2 <- MOTOR_DIR pin.
static const int MOTOR_PWM[4] = {4, 6, 0, 3};
static const int MOTOR_DIR_PIN[4] = {5, 7, 1, 10};
// Order: FL, FR, RL, RR. -1 = that motor is wired/mounted backwards, so its direction is inverted in software.
static const int MOTOR_FLIP[4] = {-1, 1, 1, 1};  // FL inverted

#define MOTOR_PWM_FREQ 20000           // 20 kHz: above hearing, no whine
#define MOTOR_PWM_BITS 10              // duty 0..1023
#define MAX_DUTY 900                   // cap, out of 1023 (keeps a 6 V N20 cooler on a 2S battery)
#define MIN_DUTY 120                   // below this an N20 just hums; treated as 0

// WHEG style: duty at the default gait speed (GAIT_FREQ_HZ). Faster gait = more duty.
#define WHEG_DUTY 600
// SWING style: duty per degree/second of commanded leg speed.
// About MAX_DUTY / (motor no-load speed in deg/s). A 100 rpm N20 = 600 deg/s -> 1.5.
#define SWING_DUTY_PER_DEG_S 1.5f

// ============================================================ Gait (default trot)
#define GAIT_FREQ_HZ 1.5f
#define GAIT_AMP_DEG 25.0f
static const float PHASE_OFFSET[4] = {0.0f, 0.5f, 0.5f, 0.0f};   // diagonal pairs together

// ============================================================ Unstick routine
#define UNSTICK_MS 1500                // rock the stuck leg back and forth this long, then resume
#define UNSTICK_AMP_DEG 15.0f
#define UNSTICK_FREQ_HZ 4.0f
#define UNSTICK_DUTY 850               // WHEG style: full-ish power each way while rocking
#define SIM_UNSTICK_FREE_CHANCE 0.6f   // SIM_SENSORS only: chance one unstick frees a jammed leg

// ============================================================ Real sensors (SIM_SENSORS 0)
// Each needs its library installed (see README). Set to 0 for anything not wired.
#define HAS_IMU 1        // MPU6050 on the body, I2C 0x68
#define HAS_CURRENT 1    // four INA219 current sensors, one in series with each motor
#define HAS_TOF 0        // one VL53L0X pointing down (copied into all four slots); the C3 has
                         // no spare pins for the XSHUT lines that four sensors would need
static const uint8_t INA219_ADDR[4] = {0x40, 0x41, 0x44, 0x45};   // set with the A0/A1 solder jumpers
