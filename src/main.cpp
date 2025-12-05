#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "wifi_setup.h"

WifiSetup *wifiSetup;

#define STEP_PIN        2
#define DIR_PIN         3


#define SDA_PIN         8
#define SCL_PIN         9
#define XSHUT_LO_PIN    4  // Pin XSHUT untuk Sensor Bawah (Homing)
#define XSHUT_HI_PIN    5  // Pin XSHUT untuk Sensor Atas (Limit Max)


#define MAX_SPEED       1500.0 // Steps/sec
#define HOMING_SPEED    -400.0 // Negative untuk mundur
#define PID_INTERVAL    10     // ms


#define LIMIT_THRESHOLD_MM  50 
#define LO_ADDR 0x30
#define HI_ADDR 0x31

float Kp = 6.0;
float Ki = 0.01;
float Kd = 25.0;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
VL53L0X sensorLo; // Sensor Bawah (Home)
VL53L0X sensorHi; // Sensor Atas (End)

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
    
    stepper.setMaxSpeed(MAX_SPEED);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    initSensors();

    homingSequence();
}

void loop() {
    if (emergencyStop) {
        stepper.setSpeed(0);
        stepper.runSpeed();
        return; // Skip logic lain
    }

    checkSafetyLimits();

    parseSerialCommand();

    unsigned long currentTime = millis();
    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        runPIDControl();
        lastPIDTime = currentTime;
    }

    stepper.runSpeed();
}

void initSensors() {
    Serial.println("Inisialisasi Sensor ToF...");

    pinMode(XSHUT_LO_PIN, OUTPUT);
    pinMode(XSHUT_HI_PIN, OUTPUT);

    digitalWrite(XSHUT_LO_PIN, LOW);
    digitalWrite(XSHUT_HI_PIN, LOW);
    delay(10);

    digitalWrite(XSHUT_LO_PIN, HIGH); // Nyalakan Sensor LO
    delay(10);
    
    if (!sensorLo.init()) {
        Serial.println("Gagal init Sensor Bawah!");
        while(1);
    }
    sensorLo.setAddress(LO_ADDR); 
    sensorLo.setTimeout(500);
    sensorLo.startContinuous();
    Serial.println("Sensor Bawah OK (Addr: 0x30)");

    digitalWrite(XSHUT_HI_PIN, HIGH); 
    delay(10);

    if (!sensorHi.init()) {
        Serial.println("Gagal init Sensor Atas!");
        while(1);
    }
    sensorHi.setAddress(HI_ADDR); 
    sensorHi.startContinuous();
    Serial.println("Sensor Atas OK (Addr: 0x31)");
}

void homingSequence() {
    Serial.println("Memulai Homing...");
    stepper.setSpeed(HOMING_SPEED);

    bool homingComplete = false;

    while (!homingComplete) {
        int distLo = sensorLo.readRangeContinuousMillimeters();

        if (sensorLo.timeoutOccurred()) {
            Serial.println("TIMEOUT Sensor LO!"); 
            // Opsional: Handle error
        }

        if (distLo < LIMIT_THRESHOLD_MM && distLo > 0 && distLo < 1200) { 
            // Kita anggap >0 dan <1200 valid (filter glitch 8190mm saat sensor bingung)
            stepper.setSpeed(0);
            homingComplete = true;
        } else {
            stepper.runSpeed();
        }
    }

    delay(500);
    stepper.setCurrentPosition(0);
    targetPosition = 0;
    isHomed = true;
    
    Serial.print("Homing Selesai. Jarak sensor saat 0: ");
    Serial.print(sensorLo.readRangeContinuousMillimeters());
    Serial.println(" mm");
}

void checkSafetyLimits() {
    // Hanya cek setiap beberapa ms agar tidak membebani I2C terlalu parah
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 50) return;
    lastCheck = millis();

    int distLo = sensorLo.readRangeContinuousMillimeters();
    int distHi = sensorHi.readRangeContinuousMillimeters();

    if (stepper.speed() < 0 && distLo < (LIMIT_THRESHOLD_MM - 10)) {
        Serial.println("BAHAYA: Terlalu dekat batas BAWAH!");
        stepper.setSpeed(0);
        targetPosition = stepper.currentPosition(); // Batalkan target
    }

    if (stepper.speed() > 0 && distHi < (LIMIT_THRESHOLD_MM - 10)) {
        Serial.println("BAHAYA: Terlalu dekat batas ATAS!");
        stepper.setSpeed(0);
        targetPosition = stepper.currentPosition(); // Batalkan target
    }
}

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