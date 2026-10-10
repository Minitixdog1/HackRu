// Everything you are likely to change lives in this file.
// Leg order is FL, FR, RL, RR in every array (front-left, front-right, rear-left, rear-right).
#pragma once
#include <stdint.h>

// ============================================================ Wi-Fi
// The ESP32 first tries to join this network (a phone hotspot works best at a
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
#define UDP_PORT 4210

// ============================================================ Timing and safety
#define TELEMETRY_HZ 50                // sensor messages per second to the PC
#define CONTROL_HZ 200                 // motor control loop rate
#define HEARTBEAT_TIMEOUT_MS 300       // no message from the PC this long -> motors off
#define PC_IDLE_MS 2000                // stop streaming if the PC has been quiet this long

// ============================================================ Mode
// 1 = no sensors or motors needed: the ESP32 makes up realistic sensor data
//     (same model as fake_robot.py), and the PC can "jam" a leg remotely.
//     Use this to test Wi-Fi + the ML pipeline with a bare ESP32.
// 0 = real hardware below.
#define SIM_SENSORS 1

// ============================================================ Hardware present (SIM_SENSORS 0)
#define HAS_IMU 1        // MPU6050 on I2C (0x68)
#define HAS_TOF 1        // VL53L0X distance sensors on I2C, one per corner, pointing down
#define HAS_CURRENT 1    // INA219 current sensor in series with each motor
#define HAS_ENCODERS 1   // N20 motors with magnetic (hall) encoders
#define HAS_PCA9685 1    // PCA9685 board drives the 4 latch servos and the 4 ToF XSHUT lines

// ============================================================ Pins (ESP32 DevKit)
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_HZ 400000

// Motor driver (DRV8833 / TB6612 in IN1/IN2 mode): two PWM pins per motor.
static const int MOTOR_IN1[4] = {16, 18, 23, 26};
static const int MOTOR_IN2[4] = {17, 19, 25, 27};

// Encoders. GPIO 34-39 have no internal pull-ups: if your encoder outputs are
// open-collector, add 10k resistors from each signal to 3.3V. Power encoders from 3.3V.
static const int ENC_A[4] = {34, 36, 32, 4};
static const int ENC_B[4] = {35, 39, 33, 13};

// PCA9685 channels and I2C addresses.
static const int SERVO_CH[4] = {0, 1, 2, 3};
static const int XSHUT_CH[4] = {4, 5, 6, 7};
#define PCA9685_ADDR 0x60   // close the A5 jumper on the board; default 0x40 clashes with an INA219
static const uint8_t INA219_ADDR[4] = {0x40, 0x41, 0x44, 0x45};
static const uint8_t TOF_ADDR[4] = {0x30, 0x31, 0x32, 0x33};   // assigned at boot

// No PCA9685? Servos can go on direct pins, and ToF sensors need their own XSHUT
// pins (-1 = not connected). With no XSHUT pins at all, a single ToF at its
// default address is read and copied into all four slots.
static const int SERVO_PIN[4] = {5, 14, 2, 15};
static const int XSHUT_PIN[4] = {-1, -1, -1, -1};

// ============================================================ Motors and joints
// Encoder counts per turn of the output shaft, full quadrature:
// magnet pulses per motor turn x 4 x gear ratio (e.g. 7 x 4 x 100 = 2800).
#define ENC_COUNTS_PER_REV 2800.0f
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_BITS 10            // duty 0..1023
#define MAX_PWM 900
#define KP 12.0f                     // PWM per degree of error
#define KD 0.4f                      // PWM per degree/second
static const int MOTOR_DIR[4] = {1, 1, 1, 1};   // flip to -1 if a leg drives the wrong way
static const int ENC_DIR[4] = {1, 1, 1, 1};     // flip to -1 if a leg's angle reads backwards

// ============================================================ Gait (default trot)
#define GAIT_FREQ_HZ 1.5f
#define GAIT_AMP_DEG 25.0f
static const float PHASE_OFFSET[4] = {0.0f, 0.5f, 0.5f, 0.0f};   // diagonal pairs together

// ============================================================ Latch servos
#define SERVO_LOCK_DEG 90
#define SERVO_RELEASE_DEG 0

// ============================================================ Unstick routine
#define UNSTICK_MS 1500              // wiggle the stuck leg this long, then resume walking
#define UNSTICK_AMP_DEG 15.0f
#define UNSTICK_FREQ_HZ 4.0f
#define SIM_UNSTICK_FREE_CHANCE 0.6f  // SIM_SENSORS only: chance one wiggle frees a jammed leg
