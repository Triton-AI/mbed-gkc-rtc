#include "USBJoystick.hpp"

namespace tritonai::gkc {

USBJoystick::USBJoystick(bool connect) : USBHID(8, 8, 0x1234, 0x0006, 0x0001, connect) {}

bool USBJoystick::update(int8_t x, int8_t y, uint8_t buttons) {
    HID_REPORT report;
    report.data[0] = x;       // X Axis (steering)
    report.data[1] = y;       // Y Axis (throttle/brake)
    report.data[2] = buttons; // Button states (if any)
    report.length = 3;
    return send(&report);
}

const uint8_t *USBJoystick::report_desc() {
    static const uint8_t reportDescriptor[] = {
        0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
        0x09, 0x04,        // Usage (Joystick)
        0xA1, 0x01,        // Collection (Application)
        0xA1, 0x02,        // Collection (Logical)

        // 2 axes (X, Y)
        0x09, 0x30,        // Usage (X)
        0x09, 0x31,        // Usage (Y)
        0x15, 0x81,        // Logical Minimum (-127)
        0x25, 0x7F,        // Logical Maximum (127)
        0x75, 0x08,        // Report Size (8 bits per axis)
        0x95, 0x02,        // Report Count (2 axes)
        0x81, 0x02,        // Input (Data,Var,Abs)

        // 8 buttons
        0x05, 0x09,        // Usage Page (Button)
        0x19, 0x01,        // Usage Minimum (Button 1)
        0x29, 0x08,        // Usage Maximum (Button 8)
        0x15, 0x00,        // Logical Minimum (0)
        0x25, 0x01,        // Logical Maximum (1)
        0x75, 0x01,        // Report Size (1 bit per button)
        0x95, 0x08,        // Report Count (8 buttons)
        0x81, 0x02,        // Input (Data,Var,Abs)

        0xC0,              // End Collection (Logical)
        0xC0               // End Collection (Application)
    };
    reportLength = sizeof(reportDescriptor);
    return reportDescriptor;
}

} // namespace tritonai::gkc
