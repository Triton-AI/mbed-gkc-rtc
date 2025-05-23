/**
 * @file vesc_can_tools.hpp
 * @brief CAN interface tools for VESC motor controllers
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Thread.h"
#include "mbed.h"
#include "config.hpp"
#include <map>

namespace tritonai::gkc {

    // External declarations for CAN interfaces
    extern CAN can1;
    extern CAN can2;

    typedef enum {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT,
        CAN_PACKET_SET_CURRENT_BRAKE,
        CAN_PACKET_SET_RPM,
        CAN_PACKET_SET_POS,
        CAN_PACKET_STATUS = 9, // ERPM, Current, Duty Cycle
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL,
        CAN_PACKET_SET_CURRENT_HANDBRAKE,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
        CAN_PACKET_STATUS_4 = 16, // Temp Fet, Temp Motor, Current In, PID position
        CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
    } CAN_PACKET_ID;

    // Function declarations
    void InitializeCan();
    void CanTransmitEid(uint32_t id, const uint8_t* data, uint8_t len);
    bool CanReceiveEid(uint32_t id, uint8_t* outData, uint8_t& outLen);
    void ProcessCanMessage(const CANMessage& msg);
    void CanRecvLoop();

    void BufferAppendInt16(uint8_t* buffer, int16_t number, int32_t* index);
    void BufferAppendInt32(uint8_t* buffer, int32_t number, int32_t* index);
    void BufferAppendFloat16(uint8_t* buffer, float number, float scale, int32_t* index);
    void BufferAppendFloat32(uint8_t* buffer, float number, float scale, int32_t* index);

    bool GetThrottleErpm(int32_t& erpm, float& speedMs);
    bool CommCanGetPos(uint8_t controllerId, float& pidPos);

    // Message sending functions
    void CommCanSetDuty(uint8_t controllerId, float duty);
    void CommCanSetCurrent(uint8_t controllerId, float current);
    void CommCanSetCurrentBrake(uint8_t controllerId, float current);
    void CommCanSetRpm(uint8_t controllerId, float rpm);
    void CommCanSetPos(uint8_t controllerId, float pos);
    void CommCanSetCurrentRel(uint8_t controllerId, float currentRel);
    void CommCanSetCurrentOffDelay(uint8_t controllerId, float current, float offDelay);
    void CommCanSetCurrentRelOffDelay(uint8_t controllerId, float currentRel, float offDelay);
    void CommCanSetCurrentBrakeRel(uint8_t controllerId, float currentRel);
    void CommCanSetHandbrake(uint8_t controllerId, float current);
    void CommCanSetHandbrakeRel(uint8_t controllerId, float currentRel);

    // Vehicle-specific functions
    void CommCanSetSpeed(float speedMs);
    void CommCanSetAngle(float steerAngle);
    float CommCanGetAngle();
    float CommCanGetSpeed();
    void CommCanSetBrakePosition(float brakePosition);

    // Utility functions
    template <typename T>
    constexpr T Clamp(const T& val, const T& min, const T& max) {
        if (val > max) {
            return max;
        } else if (val < min) {
            return min;
        } else {
            return val;
        }
    }

    template <typename S, typename D>
    constexpr D MapRange(const S& source, const S& sourceMin, const S& sourceMax,
                        const D& destMin, const D& destMax) {
        float sourceF = Clamp<S>(source, sourceMin, sourceMax);
        sourceF = (sourceF - sourceMin) / (sourceMax - sourceMin);
        return static_cast<D>(sourceF * (destMax - destMin) + destMin);
    }

    float MapSteer2Motor(float steerAngle);

} // namespace tritonai::gkc