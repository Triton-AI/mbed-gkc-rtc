/**
 * @file main.cpp
 * @brief Main entry point for the Mbed-based embedded project
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "PinNames.h"
#include "mbed.h"
#include "Controller/controller.hpp"

// Global variable for controller passthrough state
bool g_PassthroughEnabled = false;

// Button callback for toggling passthrough mode
void TogglePassthrough() {
    g_PassthroughEnabled = !g_PassthroughEnabled;
}

InterruptIn button(BUTTON1);

int main() {
    // Change user button to toggle passthrough instead of reset
    button.rise(&TogglePassthrough);
    
    new tritonai::gkc::Controller();
    
    while (true) {
        ThisThread::sleep_for(3600000ms);
    };
}