/**
 * @file USBJoystick.cpp
 * @brief Implementation of the USB HID Joystick interface
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "USBJoystick.hpp"
#include <cstring>

namespace tritonai::gkc {

    USBJoystick::USBJoystick(bool connect)
        : USBHID(connect, 8, 8, 0x1D50, 0x60A1, 0x0001) {}

    bool USBJoystick::IsConnected() {
        return configured() && USBHID::ready();
    }

    bool USBJoystick::Update(int8_t x, int8_t y, uint8_t buttons, uint8_t slider) {
        if (!IsConnected()) {
            return false;
        }
            
        HID_REPORT report;
        report.data[0] = x;       // X axis
        report.data[1] = y;       // Y axis
        report.data[2] = buttons; // Combined button bits (3 real buttons; padding ignored)
        report.data[3] = slider;  // Slider value (0-100)
        report.length = 4;
        return send(&report);
    }

    const uint8_t* USBJoystick::report_desc() {
        // This HID report descriptor defines a joystick with:
        // 2 axes (X and Y), 3 buttons, and 1 absolute slider (pot value 0-100).
        static const uint8_t reportDescriptor[] = {
            0x05, 0x01,        // Usage Page (Generic Desktop Controls)
            0x09, 0x04,        // Usage (Joystick)
            0xA1, 0x01,        // Collection (Application)
            0xA1, 0x02,        // Collection (Logical)

            // -- 2 Axes: X and Y --
            0x09, 0x30,        // Usage (X)
            0x09, 0x31,        // Usage (Y)
            0x15, 0x81,        // Logical Minimum (-127)
            0x25, 0x7F,        // Logical Maximum (127)
            0x75, 0x08,        // Report Size (8 bits per axis)
            0x95, 0x02,        // Report Count (2 axes)
            0x81, 0x02,        // Input (Data, Variable, Absolute)

            // -- 3 Buttons --
            0x05, 0x09,        // Usage Page (Button)
            0x19, 0x01,        // Usage Minimum (Button 1)
            0x29, 0x03,        // Usage Maximum (Button 3)
            0x15, 0x00,        // Logical Minimum (0)
            0x25, 0x01,        // Logical Maximum (1)
            0x75, 0x01,        // Report Size (1 bit per button)
            0x95, 0x03,        // Report Count (3 buttons)
            0x81, 0x02,        // Input (Data, Variable, Absolute)

            // -- Padding for the buttons (5 bits) --
            0x05, 0x00,        // Usage Page (Undefined) for padding so these bits are ignored
            0x75, 0x01,        // Report Size (1 bit)
            0x95, 0x05,        // Report Count (5 bits)
            0x81, 0x03,        // Input (Constant, Variable, Absolute)

            // -- 1 Absolute Slider (potentiometer as percent 0-100) --
            0x05, 0x01,        // Usage Page (Generic Desktop Controls)
            0x09, 0x36,        // Usage (Slider)
            0x15, 0x00,        // Logical Minimum (0)
            0x25, 0x64,        // Logical Maximum (100)  (0x64 == 100)
            0x75, 0x08,        // Report Size (8 bits)
            0x95, 0x01,        // Report Count (1)
            0x81, 0x02,        // Input (Data, Variable, Absolute)

            0xC0,              // End Collection (Logical)
            0xC0               // End Collection (Application)
        };
        reportLength = sizeof(reportDescriptor);
        return reportDescriptor;
    }

    const uint8_t* USBJoystick::string_iproduct_desc() {
        static const uint8_t productDesc[] = {
            0x36,               // bLength: 54 bytes
            STRING_DESCRIPTOR,  // bDescriptorType (0x03)
            'G', 0, 'K', 0, 'C', 0, ' ', 0,
            'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0, 
            ' ', 0,
            'P', 0, 'a', 0, 's', 0, 's', 0, 't', 0, 'h', 0, 'r', 0, 'o', 0, 'u', 0, 'g', 0, 'h', 0
        };
        return productDesc;
    }

    const uint8_t* USBJoystick::string_imanufacturer_desc() {
        static const uint8_t manufDesc[] = {
            0x0A,               // bLength: 10 bytes
            STRING_DESCRIPTOR,  // bDescriptorType (0x03)
            'U', 0, 'C', 0, 'S', 0, 'D', 0
        };
        return manufDesc;
    }

} // namespace tritonai::gkc