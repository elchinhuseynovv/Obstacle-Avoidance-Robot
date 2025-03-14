#ifndef DISPLAY_H
#define DISPLAY_H

#include <LiquidCrystal_I2C.h>
#include "config.h"

class Display {
private:
    LiquidCrystal_I2C lcd;
    unsigned long lastUpdate;

public:
    Display() : lcd(LCD_ADDRESS, 16, 2), lastUpdate(0) {}

    void init() {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        showWelcomeMessage();
    }

    void showWelcomeMessage() {
        lcd.setCursor(0, 0);
        lcd.print("Robot Ready!");
        lcd.setCursor(0, 1);
        lcd.print("System OK");
        delay(2000);
        lcd.clear();
    }

    void updateStatus(int distance, int speed, const char* mode) {
        if (millis() - lastUpdate > LCD_UPDATE_MS) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Dist:");
            lcd.print(distance);
            lcd.print("cm Spd:");
            lcd.print(speed);
            
            lcd.setCursor(0, 1);
            lcd.print("Mode: ");
            lcd.print(mode);
            
            lastUpdate = millis();
        }
    }

    void showError(const char* error) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ERROR:");
        lcd.setCursor(0, 1);
        lcd.print(error);
    }
};

#endif