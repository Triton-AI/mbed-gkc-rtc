/**
 * @file vesc_can_tools.hpp
 * @brief CAN interface tools for VESC motor controllers
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <map>
#include <cmath>
#include "mbed.h"
#include "config.hpp"

namespace tritonai::gkc {

    // Initialize CAN interfaces
    CAN can1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);
    CAN can2(CAN2_RX, CAN2_TX, CAN2_BAUDRATE);
    
    // Globals for storing steering sensor data
    static float g_LastSteeringAngle = 0.0f;
    static bool g_SteeringAngleReceived = false;
    static Mutex g_SteeringAngleMutex;

    // Globals for storing ERPM data
    static int32_t g_ThrottleErpm = 0;
    static float g_CalculatedSpeed = 0.0f; // Speed in m/s
    static bool g_ErpmReceived = false;
    static Mutex g_ErpmMutex;

    /**
     * @brief Transmits a CAN message with an extended ID
     */
    static void CanTransmitEid(uint32_t id, const uint8_t* data, uint8_t len) {
        CANMessage* cMsg;
        cMsg = new CANMessage(id, data, len, CANData, CANExtended);

        if (!can2.write(*cMsg)) {
            can2.reset();
            can2.frequency(CAN2_BAUDRATE);
        }
        delete cMsg;
    }

    /**
     * @brief Receives a CAN message with an extended ID
     * @return true if message received and matched, false otherwise. Data is copied to 'outData'.
     */
    static bool CanReceiveEid(uint32_t id, uint8_t* outData, uint8_t& outLen) {
        CANMessage cMsg;
        // cMsg = new CANMessage(id, data, len, CANData, CANExtended);

        if (can2.read(cMsg)) {
            if (cMsg.id == id) {
                // Process the message
                outLen = cMsg.len;
                memcpy(outData, cMsg.data, outLen);
                return true; // Message received successfully
            }
        }
        return false; // No message received or ID mismatch
    }

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

    /**
     * @brief Process a CAN message and update cached values
     */
    static void ProcessCanMessage(const CANMessage& msg) {
        // Process steering feedback (STATUS_4 packet)
        if (msg.id == (STEER_CAN_ID | ((uint32_t)CAN_PACKET_ID::CAN_PACKET_STATUS_4 << 8)) && msg.len >= 8) {
            // Extract the position data
            int16_t posRaw = (msg.data[6] << 8) | msg.data[7];
            float angleDeg = posRaw / 50.0f;

            // Convert motor angle in degrees to radians
            float angleRad = angleDeg * (M_PI / 180.0);
            
            // Reverse the conversion done in CommCanSetAngle()
            // Original conversion: motorAngle = steerAngle * 4.0 + MOTOR_OFFSET
            float steerAngleRad = (angleRad - MOTOR_OFFSET) / 4.0;
            
            // Update the cached value
            g_SteeringAngleMutex.lock();
            g_LastSteeringAngle = steerAngleRad;
            g_SteeringAngleReceived = true;
            g_SteeringAngleMutex.unlock();
        }
        // Process throttle ERPM (STATUS packet)
        else if (msg.id == (THROTTLE_CAN_ID | ((uint32_t)CAN_PACKET_ID::CAN_PACKET_STATUS << 8)) && msg.len >= 6) {
            // Extract ERPM from the first 4 bytes
            int32_t erpm = (msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
            
            // Calculate actual speed in m/s
            float speedMs = (erpm * WHEEL_CIRCUMFERENCE_M) / (NUM_MOTOR_POLES * GEAR_RATIO * 60.0);
            
            g_ErpmMutex.lock();
            g_ThrottleErpm = erpm;
            g_CalculatedSpeed = speedMs;
            g_ErpmReceived = true;
            g_ErpmMutex.unlock();
        }
    }

    /**
     * @brief CAN Rx callback function
     */
    static void CanRxCallback() {
        CANMessage msg;
        while (can2.read(msg)) {
            ProcessCanMessage(msg);
        }
    }

    /**
     * @brief Initialize CAN with callback
     */
    void InitializeCanWithCallback() {
        // Configure CAN interfaces
        can1.frequency(CAN1_BAUDRATE);
        can2.frequency(CAN2_BAUDRATE);
        
        // Set up the callback for CAN message reception
        can2.attach(&CanRxCallback);
    }

    void BufferAppendInt16(uint8_t* buffer, int16_t number, int32_t* index) {
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
    }

    void BufferAppendInt32(uint8_t* buffer, int32_t number, int32_t* index) {
        buffer[(*index)++] = number >> 24;
        buffer[(*index)++] = number >> 16;
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
    }

    void BufferAppendFloat16(uint8_t* buffer, float number, float scale, int32_t* index) {
        BufferAppendInt16(buffer, (int16_t)(number * scale), index);
    }

    void BufferAppendFloat32(uint8_t* buffer, float number, float scale, int32_t* index) {
        BufferAppendInt32(buffer, (int32_t)(number * scale), index);
    }

    /**
     * @brief Get throttle ERPM and calculated speed
     */
    bool GetThrottleErpm(int32_t& erpm, float& speedMs) {
        g_ErpmMutex.lock();
        bool dataAvailable = g_ErpmReceived;
        if (dataAvailable) {
            erpm = g_ThrottleErpm;
            speedMs = g_CalculatedSpeed;
        }
        g_ErpmMutex.unlock();
        return dataAvailable;
    }

    // Message receiving functions
    bool CommCanGetPos(uint8_t controllerId, float& pidPos) {
        uint8_t data[8];
        uint8_t len;
        uint32_t expectedId = controllerId | ((uint32_t)CAN_PACKET_STATUS_4 << 8);

        if (CanReceiveEid(expectedId, data, len) && len >= 8) {
            int16_t posRaw = (data[6] << 8) | data[7];
            pidPos = posRaw / 50.0f; // Convert to degrees
            return true;
        }
        return false;
    }

    // Message sending functions
    void CommCanSetDuty(uint8_t controllerId, float duty) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendInt32(buffer, (int32_t)(duty * 100000.0), &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, sendIndex);
    }

    void CommCanSetCurrent(uint8_t controllerId, float current) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendInt32(buffer, (int32_t)(current * 1000.0), &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, sendIndex);
    }

    void CommCanSetCurrentBrake(uint8_t controllerId, float current) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendInt32(buffer, (int32_t)(current * 1000.0), &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, sendIndex);
    }

    void CommCanSetRpm(uint8_t controllerId, float rpm) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendInt32(buffer, (int32_t)rpm, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, sendIndex);
    }

    void CommCanSetPos(uint8_t controllerId, float pos) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendInt32(buffer, (int32_t)(-1.0*pos * 1000000.0), &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, sendIndex);
    }

    void CommCanSetCurrentRel(uint8_t controllerId, float currentRel) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendFloat32(buffer, currentRel, 1e5, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, sendIndex);
    }

    void CommCanSetCurrentOffDelay(uint8_t controllerId, float current, float offDelay) {
        int32_t sendIndex = 0;
        uint8_t buffer[6];
        BufferAppendInt32(buffer, (int32_t)(current * 1000.0), &sendIndex);
        BufferAppendFloat16(buffer, offDelay, 1e3, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, sendIndex);
    }

    void CommCanSetCurrentRelOffDelay(uint8_t controllerId, float currentRel, float offDelay) {
        int32_t sendIndex = 0;
        uint8_t buffer[6];
        BufferAppendFloat32(buffer, currentRel, 1e5, &sendIndex);
        BufferAppendFloat16(buffer, offDelay, 1e3, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, sendIndex);
    }

    void CommCanSetCurrentBrakeRel(uint8_t controllerId, float currentRel) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendFloat32(buffer, currentRel, 1e5, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, sendIndex);
    }

    void CommCanSetHandbrake(uint8_t controllerId, float current) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendFloat32(buffer, current, 1e3, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, sendIndex);
    }

    void CommCanSetHandbrakeRel(uint8_t controllerId, float currentRel) {
        int32_t sendIndex = 0;
        uint8_t buffer[4];
        BufferAppendFloat32(buffer, currentRel, 1e5, &sendIndex);
        CanTransmitEid(controllerId | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, sendIndex);
    }

    void CommCanSetSpeed(float speedMs) {
        // For example: 1 mps
        // (1*5*59/22*60)/(0.254*pi) = 1,008.2471341183
        float speedToErpm = speedMs * NUM_MOTOR_POLES * GEAR_RATIO / WHEEL_CIRCUMFERENCE_M * 60.0;
        CommCanSetRpm(THROTTLE_CAN_ID, speedToErpm);
    }

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

    // Not used - Daniel
    float MapSteer2Motor(float steerAngle) {
        std::map<float, float> mapping = STEERING_MAPPING;
        int sign = steerAngle >= 0 ? 1 : -1;

        for (auto it = mapping.begin(); it != mapping.end(); it++) {
            if ((std::next(it))->second >= sign * steerAngle) {
                auto returned = sign * MapRange<float, float>(
                    sign * steerAngle, 
                    it->second,
                    std::next(it)->second, 
                    it->first,
                    std::next(it)->first
                );
                return returned;
            }
        }
        return 0;
    }

    void CommCanSetAngle(float steerAngle) {
        float motorAngle = steerAngle * 4.0 + MOTOR_OFFSET;
        float radToDeg = 180.0 / 3.14159265358979323846 * motorAngle;
        CommCanSetPos(STEER_CAN_ID, radToDeg);
    }

    float CommCanGetAngle() {
        g_SteeringAngleMutex.lock();
        float result = g_SteeringAngleReceived ? g_LastSteeringAngle : NAN;
        g_SteeringAngleMutex.unlock();
        return result;
    }

    float CommCanGetSpeed() {
        g_ErpmMutex.lock();
        float result = g_ErpmReceived ? g_CalculatedSpeed : NAN;
        g_ErpmMutex.unlock();
        return result;
    }

    void CommCanSetBrakePosition(float brakePosition) {
        brakePosition = Clamp(brakePosition, 0.0f, 1.0f);
        unsigned int pos = (unsigned int)(brakePosition * (MAX_BRAKE_VAL - MIN_BRAKE_VAL)) + MIN_BRAKE_VAL;
        
        static unsigned char buffer[8] = {0x0F, 0x4A, 0x00, 0xC0, 0, 0, 0, 0};
        buffer[2] = pos & 0xFF;
        buffer[3] = 0xC0 | ((pos >> 8) & 0x1F);

        CanTransmitEid(BRAKE_CAN_ID, buffer, 8);
    }

} // namespace tritonai::gkc