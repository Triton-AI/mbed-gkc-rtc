/**
 * @file main.cpp
 * @brief Main entry point for the Mbed-based embedded project
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "PinNames.h"
#include "mbed.h"
#include "Controller/controller.hpp"

InterruptIn button(BUTTON1);

int main() {
    button.rise(&NVIC_SystemReset);
    
    new tritonai::gkc::Controller();
    
    while (true) {
        ThisThread::sleep_for(3600000ms);
    };
}