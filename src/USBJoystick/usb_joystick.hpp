/**
 * @file USBJoystick.hpp
 * @brief USB HID Joystick interface for the controller
 */

#pragma once

#include "USBHID.h"

namespace tritonai::gkc {

    class USBJoystick : public USBHID {
    public:
        USBJoystick(bool connect = false);
        bool Update(int8_t x, int8_t y, uint8_t buttons, uint8_t dial);
        bool IsConnected();

    protected:
        // Return the report descriptor for the joystick.
        virtual const uint8_t* report_desc() override;

        // Override string descriptor functions for product and manufacturer.
        virtual const uint8_t* string_iproduct_desc() override;
        virtual const uint8_t* string_imanufacturer_desc() override;
    };

} // namespace tritonai::gkc