/**
 * LED指示器模块
 * 使用WS2812 LED显示系统状态
 */

#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <Arduino.h>
#include <FastLED.h>

// LED颜色定义
enum LEDColor {
    LED_OFF = 0,
    LED_RED = 1,
    LED_ORANGE = 2,
    LED_YELLOW = 3,
    LED_BLUE = 4,
    LED_CYAN = 5,
    LED_PURPLE = 6,
    LED_GREEN = 7,
    LED_WHITE = 8
};

class LEDIndicator {
private:
    CRGB* leds;
    uint8_t numLeds;
    uint8_t pin;
    bool initialized;

public:
    LEDIndicator(uint8_t ledPin, uint8_t ledCount);
    ~LEDIndicator();
    
    bool begin();
    void setColor(LEDColor color);
    void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
    void turnOff();
    void flash(LEDColor color, uint16_t duration_ms);
    void show();
    
    bool isReady() const { return initialized; }
};

#endif // LED_INDICATOR_H

