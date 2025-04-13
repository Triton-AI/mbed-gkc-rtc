#ifndef USBJOYSTICK_HPP_
#define USBJOYSTICK_HPP_

#include "USBHID.h"

namespace tritonai::gkc {

class USBJoystick : public USBHID {
public:
    USBJoystick(bool connect = false);
    bool update(int8_t x, int8_t y, uint8_t buttons, uint8_t dial);
    bool is_connected();

protected:
    // Return the report descriptor for the joystick.
    virtual const uint8_t *report_desc() override;

    // Override string descriptor functions for product and manufacturer.
    virtual const uint8_t *string_iproduct_desc();
    virtual const uint8_t *string_imanufacturer_desc();
};

} // namespace tritonai::gkc

#endif // USBJOYSTICK_HPP_