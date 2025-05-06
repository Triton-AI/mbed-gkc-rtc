/**
 * @file wheel_speed_sensor.hpp
 * @brief Wheel speed sensor implementation using hall effect sensor
 * 
 * @copyright Copyright 2025 Triton AI
 */

 #pragma once

 #include "Sensor/sensor_reader.hpp"
 #include "mbed.h"
 #include "Tools/logger.hpp"
 #include "config.hpp"
 
 namespace tritonai::gkc {
 
     /**
      * @brief Reads wheel speed from a hall effect sensor
      * 
      * Uses interrupts to count pulses from an ABS ring and calculates wheel speed
      */
     class WheelSpeedSensor : public ISensorProvider {
     public:
         WheelSpeedSensor(ILogger* logger);
         ~WheelSpeedSensor();
         
         bool IsReady() override;
         void PopulateReading(SensorGkcPacket& pkt) override;
         
         /**
          * @brief Get the current wheel speed in m/s
          * @return Current wheel speed
          */
         float GetSpeed() const;
         
         /**
          * @brief Get the accumulated distance in meters
          * @return Total distance traveled
          */
         float GetDistance() const;
         
         /**
          * @brief Reset the distance traveled counter
          */
         void ResetDistance();
         
     private:
         InterruptIn m_SpeedSensor;
         ILogger* m_Logger;
         Timer m_Timer;
         
         // Volatile for ISR shared variables
         volatile uint32_t m_PulseCount;
         volatile uint32_t m_LastTimerValue;
         volatile uint32_t m_CurrentTimerValue;
         volatile bool m_HasNewPulse;
         
         float m_CurrentSpeed;     // Speed in m/s
         float m_TotalDistance;    // Distance in meters
         uint32_t m_LastCalculationTime;
         uint32_t m_LastPulseCount;
         
         // Interrupt callback for pulse detection - must be static
         static void PulseDetectedStatic(void* obj);
         
         // Calculate speed based on pulse count and time
         void CalculateSpeed();
     };
 
 } // namespace tritonai::gkc