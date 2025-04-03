#ifndef USBJOYSTICK_HPP_
#define USBJOYSTICK_HPP_

#include "USBHID.h"

namespace tritonai::gkc {

class USBJoystick : public USBHID {
public:
    USBJoystick(bool connect = true);
    bool update(int8_t x, int8_t y, uint8_t buttons);

protected:
    virtual const uint8_t *report_desc() override;
};

} // namespace tritonai::gkc

#endif
