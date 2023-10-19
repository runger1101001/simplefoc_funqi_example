#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "Adafruit_NeoPixel.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"


#define FUNQI_PIN_NEOPIXEL PA10

#define FUNQI_PIN_INXL  PB0
#define FUNQI_PIN_INXH  PA14

#define FUNQI_PIN_MOSI PB5
#define FUNQI_PIN_MISO PB4
#define FUNQI_PIN_SCK  PB3
#define FUNQI_PIN_CS   PA15


Adafruit_NeoPixel pix = Adafruit_NeoPixel(1, FUNQI_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(11);

SPIClass sensorSPI = SPIClass(FUNQI_PIN_MOSI, FUNQI_PIN_MISO, FUNQI_PIN_SCK);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(FUNQI_PIN_CS);

Commander commander;
void onMotor(char* cmd) { commander.motor(&motor, cmd); }

void printDriverStatus();

uint32_t ts = 0;

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable();
    delay(5000);
    Serial.println("Welcome to FunQi Driver Firmware");

    // configure some pins
    pinMode(FUNQI_PIN_INXL, OUTPUT);
    pinMode(FUNQI_PIN_INXH, OUTPUT);
    digitalWrite(FUNQI_PIN_INXL, LOW); // disable 4th channel for now
    digitalWrite(FUNQI_PIN_INXH, LOW);

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
    driver.voltage_limit = 10.0f;
    driver.init();
    delay(1);

    Serial.print("Driver ready: ");
    Serial.println(driver.isReady()?"true":"false");
    Serial.print("Driver fault: ");
    Serial.println(driver.isFault()?"true":"false");    
    printDriverStatus();
    // Serial.println("Unlocking driver...");
    // driver.unlock();
    // STSPIN32G4PowMng powmng = driver.getPowMngRegister();
    // Serial.print("VCC voltage: ");
    // Serial.println(powmng.vcc_val);
    // Serial.println("Setting VCC to 10V");
    // powmng.vcc_val = 0b01;
    // driver.setPowMngRegister(powmng);
    // Serial.println("Locking driver...");
    // driver.lock();
    // driver.clearFaults();
    // printDriverStatus();
    // Serial.print("Driver ready: ");
    // Serial.println(driver.isReady()?"true":"false");
    // Serial.print("Driver fault: ");
    // Serial.println(driver.isFault()?"true":"false");    

    Serial.println("Initializing Sensor");
    sensor.init(&sensorSPI);
    MT6835Options1 options1 = sensor.getOptions1();
    options1.uvw_off = 0b1;
    sensor.setOptions1(options1);
    sensor.setABZEnabled(true);
    sensor.setABZResolution(16000-1); // sets 16000 PPR
    Serial.println("Sensor initialized, start angle: ");
    Serial.println(sensor.getAngle());

    Serial.println("Initializing Motor");
    motor.voltage_limit = driver.voltage_limit / 2.0f;
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SinePWM;
    motor.motion_downsample = 4;
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);
    motor.init();
    Serial.println("Motor enabled");

    pix.setPixelColor(0, 0, 52, 46);
    pix.show();
    Serial.println("Motor calibration...");
    motor.initFOC();

    Serial.println("Initializing comms");
    commander.add('M', onMotor, "Motor 1");

    pix.setPixelColor(0, 0, 52, 0);
    pix.show();
    Serial.println("Setup complete");
    ts = millis();
    motor.target = 5.0f;
}




int count = 0;

void loop(){
    uint32_t now = millis();
    if (now-ts>1000) {
        Serial.print("It/s: ");
        Serial.print(count);
        Serial.print("  Fault: ");
        Serial.print(driver.isFault()?"true":"false");
        Serial.print("  Angle: ");
        Serial.println(sensor.getAngle());
        count = 0;
        ts = now;
    }
    count++;
    motor.move();
    motor.loopFOC();
    commander.run();
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