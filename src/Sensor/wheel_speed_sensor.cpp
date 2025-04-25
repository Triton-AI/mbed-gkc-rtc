/**
 * @file wheel_speed_sensor.cpp
 * @brief Implementation of the wheel speed sensor
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "wheel_speed_sensor.hpp"
#include <string>

namespace tritonai::gkc {

    WheelSpeedSensor::WheelSpeedSensor(ILogger* logger)
        : m_SpeedSensor(WHEEL_SPEED_SENSOR_PIN),
        m_Logger(logger),
        m_PulseCount(0),
        m_LastTimerValue(0),
        m_CurrentTimerValue(0),
        m_HasNewPulse(false),
        m_CurrentSpeed(0.0f),
        m_TotalDistance(0.0f),
        m_LastCalculationTime(0),
        m_LastPulseCount(0)
    {
        // Start timer for speed calculation
        m_Timer.start();
        
        // Set up rising edge interrupt for pulse detection
        // Use static method with context pointer to avoid implicit 'this' capture
        m_SpeedSensor.rise(callback(PulseDetectedStatic, this));
        
        m_Logger->SendLog(LogPacket::Severity::INFO, 
                        "Wheel speed sensor initialized on pin " + std::to_string(WHEEL_SPEED_SENSOR_PIN));
    }

    WheelSpeedSensor::~WheelSpeedSensor() {
        // Disable interrupts on destruction
        m_SpeedSensor.rise(nullptr);
        m_Timer.stop();
    }

    // Static ISR-safe callback that receives context as parameter
    void WheelSpeedSensor::PulseDetectedStatic(void* obj) {
        WheelSpeedSensor* sensor = static_cast<WheelSpeedSensor*>(obj);
        
        // Capture time using timer directly (ISR-safe)
        sensor->m_CurrentTimerValue = sensor->m_Timer.read_us();
        
        // Increment pulse counter (atomic operation)
        sensor->m_PulseCount++;
        
        // Set flag for new pulse
        sensor->m_HasNewPulse = true;
    }

    void WheelSpeedSensor::CalculateSpeed() {
        // Use local copies of volatile variables
        uint32_t pulseCount = m_PulseCount;
        uint32_t currentTime = m_Timer.read_ms();
        uint32_t pulseDelta = pulseCount - m_LastPulseCount;
        uint32_t timeDelta = currentTime - m_LastCalculationTime;
        
        // Only calculate if enough time has passed
        if (timeDelta >= SPEED_SAMPLE_PERIOD_MS) {
            // Check for timeout - no pulses for more than 500ms means zero speed
            if (pulseDelta == 0 && (currentTime - m_CurrentTimerValue/1000) > 500) {
                // Log when speed drops to zero (only once)
                if (m_CurrentSpeed > 0.0f) {
                    m_Logger->SendLog(LogPacket::Severity::INFO, "Vehicle stopped");
                }
                m_CurrentSpeed = 0.0f;
            } 
            // Calculate speed based on pulse count if we have pulses
            else if (pulseDelta > 0) {
                // Calculate pulses per second
                float pulsesPerSecond = static_cast<float>(pulseDelta) / (timeDelta / 1000.0f);
                
                // Calculate revolutions per second
                float revsPerSecond = pulsesPerSecond / PULSES_PER_REVOLUTION;
                
                // Calculate linear speed (m/s)
                m_CurrentSpeed = revsPerSecond * WHEEL_CIRCUMFERENCE_M;
                
                // Calculate incremental distance
                float incrementalDistance = (static_cast<float>(pulseDelta) / PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_M;
                m_TotalDistance += incrementalDistance;
                
                // Log distance milestones (every meter)
                static int lastLoggedMeter = 0;
                int currentMeter = static_cast<int>(m_TotalDistance);
                if (currentMeter > lastLoggedMeter) {
                    m_Logger->SendLog(LogPacket::Severity::DEBUG, 
                                    "Distance traveled: " + std::to_string(currentMeter) + " meters");
                    lastLoggedMeter = currentMeter;
                }
            }
            
            // Update for next calculation
            m_LastPulseCount = pulseCount;
            m_LastCalculationTime = currentTime;
        }
    }

    bool WheelSpeedSensor::IsReady() {
        // Calculate speed on demand
        CalculateSpeed();
        return true;
    }

    void WheelSpeedSensor::PopulateReading(SensorGkcPacket& pkt) {
        // Make sure speed is calculated
        CalculateSpeed();
        
        // Looking at the SensorGkcPacket structure, it has wheel speeds for each wheel
        // For now, we'll use our single speed sensor for all values
        pkt.values.wheel_speed_fl = m_CurrentSpeed;
        pkt.values.wheel_speed_fr = m_CurrentSpeed;
        pkt.values.wheel_speed_rl = m_CurrentSpeed;
        pkt.values.wheel_speed_rr = m_CurrentSpeed;
        
        // Log current speed for debugging (only when moving)
        if (m_CurrentSpeed > 0.1f) {
            m_Logger->SendLog(LogPacket::Severity::DEBUG, 
                            "Current wheel speed: " + std::to_string(m_CurrentSpeed) + " m/s");
        }
    }

    float WheelSpeedSensor::GetSpeed() const {
        return m_CurrentSpeed;
    }

    float WheelSpeedSensor::GetDistance() const {
        return m_TotalDistance;
    }

} // namespace tritonai::gkc