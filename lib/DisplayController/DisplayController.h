#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <vector>

class DisplayController {
public:
    DisplayController(Adafruit_SSD1306 *display);
    void begin();
    void showMessage(const String &message);
    void runStartupAnimation();
    void update();

private:
    Adafruit_SSD1306 *display;
    String currentMessage;
    std::vector<String> messageQueue;
    bool showingMessage;
    bool inStartupAnimation;
    unsigned long lastAnimationFrameTime;
    int animationFrame;

    void drawAnimationFrame(int frame);
    void displayNextMessage();
};

#endif
