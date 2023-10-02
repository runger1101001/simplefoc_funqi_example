#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "Adafruit_NeoPixel.h"


#define FUNQI_PIN_NEOPIXEL PA10


Adafruit_NeoPixel pix = Adafruit_NeoPixel(1, FUNQI_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7);





void printDriverStatus();



void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Welcome to FunQi Driver Firmware");


    Serial.println("Initializing CORDIC");
    if (!SimpleFOC_CORDIC_Config())
        Serial.println("CORDIC init failed");


    Serial.println("Initializing LED");
    pix.begin();
    pix.setBrightness(200);
    pix.setPixelColor(0, 95, 6, 87);
    pix.show();

    Serial.println("Initializing Driver");
    driver.voltage_power_supply = 12.0f;
    driver.voltage_limit = 5.0f;
    driver.init();
    delay(1);

    Serial.print("Driver ready: ");
    Serial.println(driver.isReady()?"true":"false");
    printDriverStatus();
    Serial.println("Clearing faults, unlocking driver...");
    driver.unlock();
    driver.clearFaults();
    printDriverStatus();

    Serial.println("Initializing Motor");
    motor.voltage_limit = driver.voltage_limit / 2.0f;
    motor.controller = MotionControlType::velocity_openloop;
    motor.linkDriver(&driver);
    motor.init();

    pix.setPixelColor(0, 0, 102, 0);
    pix.show();
}

void loop(){
    // CORDIC test
    float a = millis() / 159.235668789808917f / 20;
    Serial.println(_sin(a),4);
    delay(1000);
    // motor.move(5.0f);
}




void printDriverStatus() {
    Serial.println("Driver status: ");
    STSPIN32G4Status status = driver.status();
    Serial.print("      Lock: ");
    Serial.println(status.lock?"LOCKED":"UNLOCKED");
    Serial.print("  VCC UVLO: ");
    Serial.println(status.vcc_uvlo?"YES":"NO");
    Serial.print("     VDS P: ");
    Serial.println(status.vds_p?"YES":"NO");
    Serial.print("     RESET: ");
    Serial.println(status.reset?"YES":"NO");
    Serial.print("      TSHD: ");
    Serial.println(status.thsd?"YES":"NO");
}