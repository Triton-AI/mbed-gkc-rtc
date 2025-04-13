#ifndef RC_CONTROLLER_HPP_
#define RC_CONTROLLER_HPP_

#include "config.hpp"
#include "math.h"
#include "elrs_receiver.hpp"
#include "USBJoystick/USBJoystick.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "Watchdog/watchable.hpp"
#include "Tools/logger.hpp"
#include <Thread.h>

namespace tritonai::gkc
{

struct Translation
{
    double normalize(int analogValue);
    double steering(int steerVal);
    double throttle(int throttleVal);
    double throttle_ratio(int throttleVal);
    bool keep_constant_thr(int throttleVal);
    double brake(int brakeVal);
    bool is_active(int right_toggle);
    bool isControllerPassthrough(int left_toggle);
    AutonomyMode getAutonomyMode(int rightTriVal);
    bool isLeftTriSwitchUp(int leftTriVal);
};

class RCController : public Watchable
{
    public:
    explicit RCController(GkcPacketSubscriber *sub, ILogger *logger);
    const RCControlGkcPacket& getPacket(){ 
        _is_ready = false;
        return _packet;
    }
    
    bool getIndicatorState() const;
    bool isUSBConnected() const { return _usb_connected; }

    protected:
    void update();
    Translation Map;
    Thread _rc_thread{osPriorityNormal, OS_STACK_SIZE*2, nullptr, "rc_thread"};
    void watchdog_callback();
    

    private:
    elrc_receiver _receiver;
    RCControlGkcPacket _packet{};
    bool _is_ready;
    GkcPacketSubscriber *_sub;
    float current_throttle=0.0;
    USBJoystick _joystick;
    bool _indicator_state;
    bool _usb_connected{false};
    ILogger *_logger;
};

} // namespace tritonai::gkc

#endif  // RC_CONTROLLER_HPP_