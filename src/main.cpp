/**
 * ESP32-C3 PID Camera Slider (Dual VL53L0X Version)
 * Author: Embedded Engineer Assistant
 * 
 * Update Requirements:
 * 1. STEP=GPIO2, DIR=GPIO3
 * 2. ENABLE=Hardwired GND
 * 3. Limits: Dual VL53L0X (Time-of-Flight)
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "wifi_setup.h"

WifiSetup *wifiSetup;

// --- Pin Definitions ---
#define STEP_PIN        2
#define DIR_PIN         3
// ENABLE PIN tidak didefinisikan karena hardwired ke GND

// --- I2C & Sensor Pins ---
#define SDA_PIN         8
#define SCL_PIN         9
#define XSHUT_LO_PIN    4  // Pin XSHUT untuk Sensor Bawah (Homing)
#define XSHUT_HI_PIN    5  // Pin XSHUT untuk Sensor Atas (Limit Max)

// --- Configuration ---
#define MAX_SPEED       1500.0 // Steps/sec
#define HOMING_SPEED    -400.0 // Negative untuk mundur
#define PID_INTERVAL    10     // ms

// --- Sensor Settings ---
// Jarak (mm) dimana slider dianggap sampai di ujung (Crash prevention)
#define LIMIT_THRESHOLD_MM  50 
// Alamat I2C Baru (agar tidak bentrok)
#define LO_ADDR 0x30
#define HI_ADDR 0x31

// --- PID Constants ---
float Kp = 6.0;
float Ki = 0.01;
float Kd = 25.0;

// --- Objects ---
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
VL53L0X sensorLo; // Sensor Bawah (Home)
VL53L0X sensorHi; // Sensor Atas (End)

// --- Variables ---
long targetPosition = 0;
unsigned long lastPIDTime = 0;
float previousError = 0;
float integral = 0;
bool isHomed = false;
bool emergencyStop = false;

// void appWifi(void *param);

void initSensors();
void homingSequence();
void runPIDControl();
void parseSerialCommand();
void checkSafetyLimits();

void setup() {
    Serial.begin(115200);
    delay(2000); // Waktu untuk buka serial monitor

    wifiSetup = new WifiSetup();
    wifiSetup->connectSTA();
    wifiSetup->setupWiFiSTA("wefee","wepaywefee");

    Serial.println("--- ESP32-C3 Advanced Camera Slider ---");
    
    // 1. Setup Motor
    stepper.setMaxSpeed(MAX_SPEED);
    
    // 2. Setup I2C & Sensors
    Wire.begin(SDA_PIN, SCL_PIN);
    initSensors();

    // 3. Lakukan Homing
    homingSequence();
}

void loop() {
    // Jika terjadi emergency stop (sensor mendeteksi tabrakan), hentikan loop
    if (emergencyStop) {
        stepper.setSpeed(0);
        stepper.runSpeed();
        return; // Skip logic lain
    }

    // 1. Safety Check (Baca sensor terus menerus)
    checkSafetyLimits();

    // 2. Baca Input User
    parseSerialCommand();

    // 3. Kalkulasi PID
    unsigned long currentTime = millis();
    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        runPIDControl();
        lastPIDTime = currentTime;
    }

    // 4. Gerakkan Motor
    stepper.runSpeed();
}

// --- Inisialisasi Dual VL53L0X ---
void initSensors() {
    Serial.println("Inisialisasi Sensor ToF...");

    pinMode(XSHUT_LO_PIN, OUTPUT);
    pinMode(XSHUT_HI_PIN, OUTPUT);

    // Reset kedua sensor (Matikan keduanya)
    digitalWrite(XSHUT_LO_PIN, LOW);
    digitalWrite(XSHUT_HI_PIN, LOW);
    delay(10);

    // --- Konfigurasi Sensor Bawah (LO) ---
    digitalWrite(XSHUT_LO_PIN, HIGH); // Nyalakan Sensor LO
    delay(10);
    
    if (!sensorLo.init()) {
        Serial.println("Gagal init Sensor Bawah!");
        while(1);
    }
    sensorLo.setAddress(LO_ADDR); // Ganti alamat ke 0x30
    sensorLo.setTimeout(500);
    sensorLo.startContinuous();
    Serial.println("Sensor Bawah OK (Addr: 0x30)");

    // --- Konfigurasi Sensor Atas (HI) ---
    digitalWrite(XSHUT_HI_PIN, HIGH); // Nyalakan Sensor HI
    delay(10);

    if (!sensorHi.init()) {
        Serial.println("Gagal init Sensor Atas!");
        while(1);
    }
    sensorHi.setAddress(HI_ADDR); // Ganti alamat ke 0x31
    sensorHi.startContinuous();
    Serial.println("Sensor Atas OK (Addr: 0x31)");
}

// --- Homing dengan Sensor Jarak ---
void homingSequence() {
    Serial.println("Memulai Homing...");
    stepper.setSpeed(HOMING_SPEED);

    bool homingComplete = false;

    while (!homingComplete) {
        // Baca jarak sensor bawah
        int distLo = sensorLo.readRangeContinuousMillimeters();
        
        // Debug posisi (opsional, hati-hati memperlambat loop)
        // Serial.print("Jarak Homing: "); Serial.println(distLo);

        if (sensorLo.timeoutOccurred()) {
            Serial.println("TIMEOUT Sensor LO!"); 
            // Opsional: Handle error
        }

        // Jika jarak sudah dekat dengan batas (misal < 50mm)
        if (distLo < LIMIT_THRESHOLD_MM && distLo > 0 && distLo < 1200) { 
            // Kita anggap >0 dan <1200 valid (filter glitch 8190mm saat sensor bingung)
            stepper.setSpeed(0);
            homingComplete = true;
        } else {
            stepper.runSpeed();
        }
    }

    delay(500);
    // Set posisi 0 di sini
    stepper.setCurrentPosition(0);
    targetPosition = 0;
    isHomed = true;
    
    Serial.print("Homing Selesai. Jarak sensor saat 0: ");
    Serial.print(sensorLo.readRangeContinuousMillimeters());
    Serial.println(" mm");
}

// --- Safety Monitor ---
void checkSafetyLimits() {
    // Hanya cek setiap beberapa ms agar tidak membebani I2C terlalu parah
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 50) return;
    lastCheck = millis();

    int distLo = sensorLo.readRangeContinuousMillimeters();
    int distHi = sensorHi.readRangeContinuousMillimeters();

    // Batas Bawah (Jika bergerak mundur dan terlalu dekat)
    // Logika: Jika kita bergerak ke negatif (mundur) DAN jarak < threshold
    if (stepper.speed() < 0 && distLo < (LIMIT_THRESHOLD_MM - 10)) {
        Serial.println("BAHAYA: Terlalu dekat batas BAWAH!");
        stepper.setSpeed(0);
        targetPosition = stepper.currentPosition(); // Batalkan target
    }

    // Batas Atas (Jika bergerak maju dan terlalu dekat)
    if (stepper.speed() > 0 && distHi < (LIMIT_THRESHOLD_MM - 10)) {
        Serial.println("BAHAYA: Terlalu dekat batas ATAS!");
        stepper.setSpeed(0);
        targetPosition = stepper.currentPosition(); // Batalkan target
    }
}

// --- PID Control ---
void runPIDControl() {
    if (!isHomed) return;

    long currentPosition = stepper.currentPosition();
    float error = targetPosition - currentPosition;

    if (abs(error) < 5) {
        error = 0;
        integral = 0;
        stepper.setSpeed(0);
        return;
    }

    float P = Kp * error;
    integral += error;
    integral = constrain(integral, -2000, 2000); 
    float I = Ki * integral;
    float D = Kd * (error - previousError);
    previousError = error;

    float pidOutput = P + I + D;

    if (pidOutput > MAX_SPEED) pidOutput = MAX_SPEED;
    if (pidOutput < -MAX_SPEED) pidOutput = -MAX_SPEED;

    stepper.setSpeed(pidOutput);
}

// --- Serial Parser ---
void parseSerialCommand() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("STOP")) {
            emergencyStop = true;
            Serial.println("EMERGENCY STOP!");
        } 
        else if (input.equalsIgnoreCase("RESET")) {
            emergencyStop = false;
            Serial.println("System Reset.");
        }
        else if (input.length() > 0 && !emergencyStop) {
            long newPos = input.toInt();
            targetPosition = newPos;
            previousError = 0; 
            Serial.print("Target: "); Serial.println(targetPosition);
        }
    }
}