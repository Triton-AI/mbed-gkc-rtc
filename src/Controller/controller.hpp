/**
 * @file controller.hpp
 * @brief Main system controller
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "Comm/comm.hpp"
#include "tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "Watchdog/watchdog.hpp"
#include "Sensor/sensor_reader.hpp"
#include "Actuation/actuation_controller.hpp"
#include "RCController/RCController.hpp"
#include "StateMachine/state_machine.hpp"
#include "Sensor/brake_pressure_sensor.hpp"
#include "Sensor/wheel_speed_sensor.hpp"
#include <chrono>

namespace tritonai::gkc {

    class Controller :
        public GkcPacketSubscriber,
        public Watchable,
        public ILogger,
        public GkcStateMachine
    {
    public:
        Controller();

        void AgxHeartbeat();
        void UpdateLights();

    protected:
        // GkcPacketSubscriber API
        void packet_callback(const Handshake1GkcPacket& packet);
        void packet_callback(const Handshake2GkcPacket& packet);
        void packet_callback(const GetFirmwareVersionGkcPacket& packet);
        void packet_callback(const FirmwareVersionGkcPacket& packet);
        void packet_callback(const ResetRTCGkcPacket& packet);
        void packet_callback(const HeartbeatGkcPacket& packet);
        void packet_callback(const ConfigGkcPacket& packet);
        void packet_callback(const StateTransitionGkcPacket& packet);
        void packet_callback(const ControlGkcPacket& packet);
        void packet_callback(const SensorGkcPacket& packet);
        void packet_callback(const Shutdown1GkcPacket& packet);
        void packet_callback(const Shutdown2GkcPacket& packet);
        void packet_callback(const LogPacket& packet);
        void packet_callback(const RCControlGkcPacket& packet);

        // ILogger API
        void SendLog(const LogPacket::Severity& severity, 
                    const std::string& what) override;
        LogPacket::Severity m_Severity;

        // Watchable API
        void WatchdogCallback();

        // GkcStateMachine API
        StateTransitionResult OnInitialize(const GkcLifecycle& lastState) override;
        StateTransitionResult OnDeactivate(const GkcLifecycle& lastState) override;
        StateTransitionResult OnActivate(const GkcLifecycle& lastState) override;
        StateTransitionResult OnEmergencyStop(const GkcLifecycle& lastState) override;
        StateTransitionResult OnReinitialize(const GkcLifecycle& lastState) override;

    private:
        CommManager m_Comm;
        Watchdog m_Watchdog;
        SensorReader m_SensorReader;
        ActuationController m_Actuation;
        RCController m_RcController;
        BrakePressureSensor m_BrakePressureSensor;
        WheelSpeedSensor m_WheelSpeedSensor;

        Thread m_KeepAliveThread{osPriorityNormal, OS_STACK_SIZE, nullptr, "keep_alive_thread"};
        bool m_RcCommanding{false};
        std::chrono::time_point<std::chrono::steady_clock> m_LastRcCommand = std::chrono::steady_clock::now();
        Watchable m_RcHeartbeat;
        void OnRcDisconnect();
        bool m_StopOnRcDisconnect{true};
        void SetActuationValues(float throttle, float steering, float brake);
        DigitalOut m_Led{LED1};
        DigitalOut m_TowerLightRed{TOWER_LIGHT_RED, 0};
        DigitalOut m_TowerLightYellow{TOWER_LIGHT_YELLOW, 0};
        DigitalOut m_TowerLightGreen{TOWER_LIGHT_GREEN, 0};
        DigitalOut m_ThrottleVescDisable{THROTTLE_VESC_DISABLE_PIN, 0};
        DigitalOut m_SteeringVescDisable{STEERING_VESC_DISABLE_PIN, 0};
        
        // Light control variables
        bool m_RcConnected{false};
        bool m_EmergencyActive{false};
        AutonomyMode m_CurrentAutonomyMode{AUTONOMOUS};
        chrono::time_point<chrono::steady_clock> m_LastLightToggle = chrono::steady_clock::now();
        bool m_LightState{false}; // For flashing
    };

} // namespace tritonai::gkc