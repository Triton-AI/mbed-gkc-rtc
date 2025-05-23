/**
 * @file vesc_can_tools.cpp
 * @brief CAN interface tools for VESC motor controllers - Implementation
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "vesc_can_tools.hpp"
#include <cstring>

namespace tritonai::gkc {

    CAN can1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);
    CAN can2(CAN2_RX, CAN2_TX, CAN2_BAUDRATE);
    
    static float g_LastSteeringAngle = 0.0f;
    static bool g_SteeringAngleReceived = false;
    static Mutex g_SteeringAngleMutex;

    static int32_t g_ThrottleErpm = 0;
    static float g_CalculatedSpeed = 0.0f; // Speed in m/s
    static bool g_ErpmReceived = false;
    static Mutex g_ErpmMutex;

    void CanTransmitEid(uint32_t id, const uint8_t* data, uint8_t len) {
        CANMessage* cMsg;
        cMsg = new CANMessage(id, data, len, CANData, CANExtended);

        if (!can2.write(*cMsg)) {
            can2.reset();
            can2.frequency(CAN2_BAUDRATE);
        }
        delete cMsg;
    }

    bool CanReceiveEid(uint32_t id, uint8_t* outData, uint8_t& outLen) {
        CANMessage cMsg;

        if (can2.read(cMsg)) {
            if (cMsg.id == id) {
                outLen = cMsg.len;
                memcpy(outData, cMsg.data, outLen);
                return true;
            }
        }
        return false;
    }

    void ProcessCanMessage(const CANMessage& msg) {
        // Process steering feedback (STATUS_4 packet)
        if (msg.id == (STEER_CAN_ID | ((uint32_t)CAN_PACKET_ID::CAN_PACKET_STATUS_4 << 8)) && msg.len >= 8) {
            // This calculation needs to be verified

            // Convert motor angle in degrees to radians
            int16_t posRaw = (msg.data[6] << 8) | msg.data[7];
            float angleDeg = std::abs((posRaw / 50.0f) - 360.0);
            float angleRad = angleDeg * (M_PI / 180.0);
            
            // Reverse the conversion done in CommCanSetAngle()
            float steerAngleRad = (angleRad - MOTOR_OFFSET) / 4.0f;

            if (steerAngleRad > 1.0f) {
                steerAngleRad = 2.0f - steerAngleRad;
            }
            
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

    void CanRecvLoop() {
        CANMessage msg;
        while (true) {
            while (can2.read(msg)) {
                ProcessCanMessage(msg);
            }
            // small sleep to yield CPU (tweak as needed)
            ThisThread::sleep_for(1ms);
        }
    }

    void InitializeCan() {
        can1.frequency(CAN1_BAUDRATE);
        can2.frequency(CAN2_BAUDRATE);

        static Thread canThread(osPriorityNormal,
                                OS_STACK_SIZE,
                                nullptr,
                                "can_recv_thread");
        canThread.start(callback(CanRecvLoop));
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

    bool CommCanGetPos(uint8_t controllerId, float& pidPos) {
        uint8_t data[8];
        uint8_t len;
        uint32_t expectedId = controllerId | ((uint32_t)CAN_PACKET_STATUS_4 << 8);

        if (CanReceiveEid(expectedId, data, len) && len >= 8) {
            int16_t posRaw = (data[6] << 8) | data[7];
            pidPos = posRaw / 50.0f;
            return true;
        }
        return false;
    }

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
        float speedToErpm = speedMs * NUM_MOTOR_POLES * GEAR_RATIO / WHEEL_CIRCUMFERENCE_M * 60.0;
        CommCanSetRpm(THROTTLE_CAN_ID, speedToErpm);
    }

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
        float radToDeg = 180.0 / M_PI * motorAngle;
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