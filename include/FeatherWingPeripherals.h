//
// Created by jwscoggins on 6/4/21.
//

#ifndef I960SXCHIPSET_FEATHERWINGPERIPHERALS_H
#define I960SXCHIPSET_FEATHERWINGPERIPHERALS_H
#include "MemoryThing.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADT7410.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class AdafruitLSM6DSOXThing : public IOSpaceThing {
public:
    AdafruitLSM6DSOXThing(Address base) : IOSpaceThing(base, base + 0x100) {
        /// @todo implement registers
    }
    ~AdafruitLSM6DSOXThing() override = default;
    void
    begin() noexcept override {
            if (!lsm6ds_.begin_I2C()) {
                signalHaltState(F("FAILED TO BRING UP LSM6DS"));
            }
    }
private:
    Adafruit_LSM6DSOX lsm6ds_;
};

class AdafruitLIS3MDLThing : public IOSpaceThing {
public:
    AdafruitLIS3MDLThing(Address base) : IOSpaceThing(base, base + 0x100) {

    }
    ~AdafruitLIS3MDLThing() override = default;
    void
    begin() noexcept override {
            if (!lis3mdl_.begin_I2C()) {
                signalHaltState(F("FAILED TO BRING UP LIS3MDL"));
            }
    }
private:
    Adafruit_LIS3MDL lis3mdl_;
};

// for the feather m0 only

class AdafruitADT7410Thing : public IOSpaceThing {
public:
    AdafruitADT7410Thing(Address base) : IOSpaceThing(base, base + 0x100) { }
    ~AdafruitADT7410Thing() override = default;
    void
    begin() noexcept override {
            if (!tempSensor_.begin()) {
                signalHaltState(F("ADT7410 Bringup Failure"));
            }
            // sensor takes 250 ms to get readings
            delay(250);
    }
private:
    Adafruit_ADT7410 tempSensor_;
};

class AdafruitADXL343Thing : public IOSpaceThing {
public:
    explicit AdafruitADXL343Thing(Address base) : IOSpaceThing(base, base + 0x100), accel1_(12345) { }
    ~AdafruitADXL343Thing() override = default;
    void
    begin() noexcept override {
            if (!accel1_.begin()) {
                signalHaltState(F("ADXL343 Bringup Failure"));
            }
            // configure for your project
            accel1_.setRange(ADXL343_RANGE_16_G);
            // sensor takes 250 ms to get readings
            delay(250);
    }
private:
    Adafruit_ADXL343 accel1_;
};
class AdafruitFeatherWingDisplay128x32Thing : public IOSpaceThing {
public:
    explicit AdafruitFeatherWingDisplay128x32Thing(Address base) : IOSpaceThing(base, base + 0x100), display_(128, 32, &Wire) { }
    ~AdafruitFeatherWingDisplay128x32Thing() override = default;
    void
    begin() noexcept override {
            // sensor takes 250 ms to get readings
            Serial.println(F("Setting up OLED Featherwing"));
            display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);
            Serial.println(F("OLED begun"));
            display_.display();
            delay(1000);
            display_.clearDisplay();
            display_.display();
            // I have cut the pins for the buttons on the featherwing display
            display_.setTextSize(2);
            display_.setTextColor(SSD1306_WHITE);
            display_.setCursor(0, 0);
            display_.println(F("i960Sx!"));
            display_.display();
            Serial.println(F("Done setting up OLED Featherwing"));
            oledDisplaySetup = true;
    }
private:
    Adafruit_SSD1306 display_;
};

#endif //I960SXCHIPSET_FEATHERWINGPERIPHERALS_H
