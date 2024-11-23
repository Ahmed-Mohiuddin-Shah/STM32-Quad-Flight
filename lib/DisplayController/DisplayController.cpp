#include "DisplayController.h"

DisplayController::DisplayController(Adafruit_SSD1306 *display)
    : display(display), showingMessage(false), inStartupAnimation(true),
      lastAnimationFrameTime(0), animationFrame(0) {}

void DisplayController::begin() {
    display->begin(SSD1306_SWITCHCAPVCC, 0x3C); // Adjust the I2C address if needed
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->display();
}

void DisplayController::showMessage(const String &message) {
    messageQueue.push_back(message);

    // If no message is currently being shown or the startup animation is done, show the new message immediately.
    if (!showingMessage && !inStartupAnimation) {
        displayNextMessage();
    }
}

void DisplayController::runStartupAnimation() {
    inStartupAnimation = true;
    animationFrame = 0;
    lastAnimationFrameTime = millis();
}

void DisplayController::update() {
    // Handle the startup animation
    if (inStartupAnimation) {
        if (millis() - lastAnimationFrameTime >= 100) { // Adjust timing as needed
            drawAnimationFrame(animationFrame++);
            lastAnimationFrameTime = millis();

            if (animationFrame > 10) { // Adjust the number of frames
                inStartupAnimation = false;
                displayNextMessage(); // Show the next message after the animation ends
            }
        }
    }
    else {
        displayNextMessage();
    }
}

void DisplayController::drawAnimationFrame(int frame) {
    display->clearDisplay();
    display->setCursor(0, 0);

    // Simple animation example: Draw a line moving across the screen
    display->drawLine(0, frame * 6, 128, frame * 6, SSD1306_WHITE);
    display->setCursor(0, 50);
    display->print("Initializing...");
    display->display();
}

void DisplayController::displayNextMessage() {
    if (!messageQueue.empty()) {
        showingMessage = true;
        currentMessage = messageQueue.back(); // Show the most recent message
        messageQueue.clear(); // Clear the queue after displaying

        // Display the message
        display->clearDisplay();
        display->setCursor(0, 0);
        display->print(currentMessage);
        display->display();
    } else {
        showingMessage = false;
    }
}
