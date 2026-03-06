/**
 * LED指示器实现
 */

#include "LEDIndicator.h"

LEDIndicator::LEDIndicator(uint8_t ledPin, uint8_t ledCount) 
    : pin(ledPin), numLeds(ledCount), initialized(false) {
    leds = new CRGB[numLeds];
}

LEDIndicator::~LEDIndicator() {
    if (leds) {
        delete[] leds;
    }
}

bool LEDIndicator::begin() {
    // 根据引脚号初始化FastLED
    // 注意：ESP32-S3的板载LED通常在GPIO48
    FastLED.addLeds<WS2812, 48, GRB>(leds, numLeds);
    FastLED.setBrightness(50);  // 设置亮度 (0-255)
    
    // 清空LED
    turnOff();
    initialized = true;
    
    return true;
}

void LEDIndicator::setColor(LEDColor color) {
    switch (color) {
        case LED_RED:
            setColorRGB(255, 0, 0);
            break;
        case LED_ORANGE:
            setColorRGB(255, 128, 0);
            break;
        case LED_YELLOW:
            setColorRGB(255, 255, 0);
            break;
        case LED_BLUE:
            setColorRGB(0, 0, 255);
            break;
        case LED_CYAN:
            setColorRGB(0, 255, 255);
            break;
        case LED_PURPLE:
            setColorRGB(128, 0, 255);
            break;
        case LED_GREEN:
            setColorRGB(0, 255, 0);
            break;
        case LED_WHITE:
            setColorRGB(255, 255, 255);
            break;
        case LED_OFF:
        default:
            turnOff();
            break;
    }
}

void LEDIndicator::setColorRGB(uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized) return;
    
    for (uint8_t i = 0; i < numLeds; i++) {
        leds[i] = CRGB(r, g, b);
    }
    FastLED.show();
}

void LEDIndicator::turnOff() {
    if (!initialized) return;
    
    for (uint8_t i = 0; i < numLeds; i++) {
        leds[i] = CRGB::Black;
    }
    FastLED.show();
}

void LEDIndicator::flash(LEDColor color, uint16_t duration_ms) {
    if (!initialized) return;
    
    setColor(color);
    delay(duration_ms);
    turnOff();
}

void LEDIndicator::show() {
    if (!initialized) return;
    FastLED.show();
}

