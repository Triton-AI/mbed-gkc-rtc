/**
 * @file controller.cpp
 * @brief Implementation of the main system controller
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "Controller/controller.hpp"
#include "config.h"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/version.hpp"
#include <chrono>
#include <iostream>

namespace tritonai::gkc {

    void Controller::UpdateLights() {
        auto now = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - m_LastLightToggle).count();
        
        // Flash at 1Hz (toggle every 500ms)
        if (elapsed >= 500) {
            m_LightState = !m_LightState;
            m_LastLightToggle = now;
        }
        
        // Turn off all lights first
        m_TowerLightRed = 0;
        m_TowerLightYellow = 0;
        m_TowerLightGreen = 0;
        
        // Emergency state takes precedence - flash red at 1Hz
        if (m_EmergencyActive) {
            m_TowerLightRed = m_LightState;
            return;
        }
        
        // If RC is not connected, flash yellow at 1Hz
        if (!m_RcConnected) {
            m_TowerLightYellow = m_LightState;
            return;
        }
        
        // Check for controller passthrough mode
        bool passthroughMode = (m_CurrentAutonomyMode == AUTONOMOUS && 
                                m_RcController.GetIndicatorState());
        
        // Check USB connection when in passthrough mode
        if (passthroughMode) {
            if (m_RcController.IsUSBConnected()) {
                // USB connected - solid orange for normal passthrough
                m_TowerLightRed = 1;
                m_TowerLightYellow = 1;
            } else {
                // USB not connected - flash orange to indicate issue
                m_TowerLightRed = m_LightState;
                m_TowerLightYellow = m_LightState;
            }
            return;
        }
        
        // If RC is connected, set lights based on autonomy mode
        switch (m_CurrentAutonomyMode) {
        case AUTONOMOUS:
            m_TowerLightGreen = 1; // Solid green for full autonomous
            break;
        case AUTONOMOUS_OVERRIDE:
            m_TowerLightYellow = 1; // Solid yellow for autonomous with override
            break;
        case MANUAL:
            m_TowerLightRed = 1; // Solid red for manual only
            break;
        default:
            m_TowerLightYellow = m_LightState; // Flash yellow for unknown state
            break;
        }
    }

    void Controller::AgxHeartbeat() {
        HeartbeatGkcPacket packet;
        std::string state;
        std::string oldState;

        //TODO: (Moises) TEMP
        GkcStateMachine::Initialize();
        
        while(true) {
            ThisThread::sleep_for(std::chrono::milliseconds(100));

            m_Led = !m_Led;
            packet.rolling_counter++;
            packet.state = GetState();
            m_Comm.Send(packet);
            this->IncCount();
            
            UpdateLights();
            
            switch(GetState()) {
            case GkcLifecycle::Uninitialized:
                state = "Uninitialized";
                break;
            case GkcLifecycle::Initializing:
                state = "Initializing";
                break;
            case GkcLifecycle::Inactive:
                state = "Inactive";
                break;
            case GkcLifecycle::Active:
                state = "Active";
                break;
            case GkcLifecycle::Emergency:
                state = "Emergency";
                break;
            default:
                state = "Unknown";
                break;
            }

            if(state != oldState) {
                oldState = state;
                SendLog(LogPacket::Severity::WARNING, "Controller state: " + state);
            }
        }
    }

    void Controller::OnRcDisconnect() {
        SendLog(LogPacket::Severity::INFO, "Controller heartbeat lost");
        m_RcConnected = false; // Mark RC as disconnected for the light system
        
        if(GetState() != GkcLifecycle::Active) {
            SendLog(LogPacket::Severity::INFO, "Controller is not active, ignoring RC controller heartbeat lost");
            return;
        }

        SendLog(LogPacket::Severity::FATAL, "RC controller heartbeat lost");
        SetActuationValues(0.0, 0.0, EMERGENCY_BRAKE_PRESSURE); // Set the actuation values to stop the car
        EmergencyStop();
    }

    // Controller initialization
    Controller::Controller() :
        Watchable(DEFAULT_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_CONTROLLER_POLL_LOST_TOLERANCE_MS, "Controller"),
        GkcStateMachine(),
        m_Severity(LogPacket::Severity::FATAL),
        m_Comm(this, this),
        m_Watchdog(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS, DEFAULT_WD_WAKEUP_INTERVAL_MS, this),
        m_SensorReader(this),
        m_Actuation(this),
        m_RcController(this, this),
        m_BrakePressureSensor(this),
        m_WheelSpeedSensor(this),
        m_RcHeartbeat(DEFAULT_RC_HEARTBEAT_INTERVAL_MS, DEFAULT_RC_HEARTBEAT_LOST_TOLERANCE_MS, "RCControllerHeartBeat")
    {
        Attach(callback(this, &Controller::WatchdogCallback));
        m_KeepAliveThread.start(callback(this, &Controller::AgxHeartbeat));

        // Add all objects to the watchlist
        m_Watchdog.AddToWatchlist(this);
        m_Watchdog.AddToWatchlist(&m_Comm);
        m_Watchdog.AddToWatchlist(&m_SensorReader);
        m_Watchdog.AddToWatchlist(&m_RcController);
        
        if(m_StopOnRcDisconnect) {
            m_RcHeartbeat.Attach(callback(this, &Controller::OnRcDisconnect));
            m_Watchdog.AddToWatchlist(&m_RcHeartbeat);
        }

        m_SensorReader.RegisterProvider(&m_BrakePressureSensor);
        m_SensorReader.RegisterProvider(&m_WheelSpeedSensor); 

        SendLog(LogPacket::Severity::INFO, "Controller initialized");
    }

    void Controller::WatchdogCallback() {
        SendLog(LogPacket::Severity::FATAL, "Controller watchdog trigger");
        NVIC_SystemReset();
    }

    // ILogger API IMPLEMENTATION
    // TODO: SendLog partially implemented, complete the implementation
    void Controller::SendLog(const LogPacket::Severity& severity, const std::string& what) {
        if(severity == LogPacket::Severity::FATAL && m_Severity <= severity)
            std::cerr << "Fatal: " << what << std::endl;
        else if(severity == LogPacket::Severity::ERROR && m_Severity <= severity)
            std::cerr << "Error: " << what << std::endl;
        else if(severity == LogPacket::Severity::WARNING && m_Severity <= severity)
            std::cerr << "Warning: " << what << std::endl;
        else if(severity == LogPacket::Severity::INFO && m_Severity <= severity)
            std::cout << "Info: " << what << std::endl;
        else if(severity == LogPacket::Severity::DEBUG && m_Severity <= severity)
            std::cout << "Debug: " << what << std::endl;
    }

    // PACKET CALLBACKS API IMPLEMENTATION
    void Controller::packet_callback(const Handshake1GkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "Handshake1GkcPacket received");
        Handshake2GkcPacket response;
        response.seq_number = packet.seq_number + 1;
        m_Comm.Send(response);
    }

    void Controller::packet_callback(const Handshake2GkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "Handshake2GkcPacket received, should not happen, ignoring.");
    }

    void Controller::packet_callback(const GetFirmwareVersionGkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "GetFirmwareVersionGkcPacket received");
        FirmwareVersionGkcPacket response;
        response.major = GkcPacketLibVersion::MAJOR;
        response.minor = GkcPacketLibVersion::MINOR;
        response.patch = GkcPacketLibVersion::PATCH;
        m_Comm.Send(response);
    }

    void Controller::packet_callback(const FirmwareVersionGkcPacket& packet) {
        SendLog(LogPacket::Severity::INFO, "FirmwareVersionGkcPacket received, should not happen, ignoring.");
    }

    void Controller::packet_callback(const ResetRTCGkcPacket& packet) {
        SendLog(LogPacket::Severity::FATAL, "ResetRTCGkcPacket received");
        NVIC_SystemReset();
    }

    void Controller::packet_callback(const HeartbeatGkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "HeartbeatGkcPacket received");
    }

    void Controller::packet_callback(const ConfigGkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "ConfigGkcPacket received");
    }

    void Controller::packet_callback(const StateTransitionGkcPacket& packet) {
        SendLog(LogPacket::Severity::INFO, "StateTransitionGkcPacket received");

        switch(packet.requested_state) {
        case GkcLifecycle::Uninitialized:
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Uninitialized");
            EmergencyStop();
            break;
        case GkcLifecycle::Initializing:
            SendLog(LogPacket::Severity::INFO, "Controller asked to transition to Initializing, ignoring");
            break;
        case GkcLifecycle::Inactive:
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Inactive");
            if(GetState() == GkcLifecycle::Uninitialized)
                Initialize();
            else if(GetState() == GkcLifecycle::Active)
                GkcStateMachine::Deactivate();
            break;
        case GkcLifecycle::Active:
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Active");
            GkcStateMachine::Activate();
            break;
        case GkcLifecycle::Emergency:
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Emergency");
            EmergencyStop();
            break;
        default:
            SendLog(LogPacket::Severity::ERROR, "Controller asked to transition to unknown state, ignoring");
            break;
        }
    }

      // TODO: Implement the control packet callback, partially done
    void Controller::packet_callback(const ControlGkcPacket& packet) {
        SendLog(LogPacket::Severity::INFO, "ControlGkcPacket received");
        if(GetState() != GkcLifecycle::Active) {
            SendLog(LogPacket::Severity::INFO, "Controller is not active, ignoring ControlGkcPacket");
            return;
        }

        if(m_RcCommanding) {
            SendLog(LogPacket::Severity::WARNING, "RC is commanding, ignoring ControlGkcPacket");
            return;
        }

        SendLog(LogPacket::Severity::INFO, "ControlGkcPacket received: throttle: " + 
                std::to_string((int)(packet.throttle * 100)) + "%, " +
                "steering: " + std::to_string((int)(packet.steering * 100)) + "%, " +
                "brake: " + std::to_string((int)(packet.brake * 100)) + "%");
        
        SetActuationValues(packet.throttle, packet.steering, packet.brake);
    }

    // TODO: Implement the sensor packet callback
    void Controller::packet_callback(const SensorGkcPacket& packet) {
        SendLog(LogPacket::Severity::DEBUG, "SensorGkcPacket received");

        SendLog(LogPacket::Severity::INFO, 
            "Current brake pressure: " + std::to_string(packet.values.brake_pressure) + " PSI");
    }

    // TODO: Implement the shutdown1 packet callbacks
    void Controller::packet_callback(const Shutdown1GkcPacket& packet) {
        SendLog(LogPacket::Severity::INFO, "Shutdown1GkcPacket received");
    }

    // TODO: Implement the shutdown2 packet callbacks
    void Controller::packet_callback(const Shutdown2GkcPacket& packet) {
        SendLog(LogPacket::Severity::INFO, "Shutdown2GkcPacket received");
    }

    // TODO: Implement the log packet callback
    void Controller::packet_callback(const LogPacket& packet) {
        SendLog(packet.level, packet.what);
    }

    void Controller::packet_callback(const RCControlGkcPacket& packet) {
        m_RcHeartbeat.IncCount();
        m_RcConnected = true;
        
        // Update light tower state based on emergency stop status
        m_EmergencyActive = !packet.is_active;
        
        // Update autonomy mode for light system
        m_CurrentAutonomyMode = packet.autonomy_mode;

        if (GetState() == GkcLifecycle::Uninitialized) {
            SendLog(LogPacket::Severity::WARNING, "Controller is uninitialized, ignoring RCControlGkcPacket");
            return;
        }

        if(!packet.is_active && GetState() != GkcLifecycle::Inactive) {
            SendLog(LogPacket::Severity::FATAL, "RCControlGkcPacket is not active, calling EmergencyStop()");
            SetActuationValues(0.0, 0.0, packet.brake);
            EmergencyStop();
            return;
        }

        if(!packet.is_active && GetState() == GkcLifecycle::Inactive) {
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Inactive");
            SetActuationValues(0.0, 0.0, packet.brake);
            return;
        }

        if(packet.is_active && GetState() == GkcLifecycle::Inactive) {
            SendLog(LogPacket::Severity::INFO, "Controller transitioning to Active");
            GkcStateMachine::Activate();
            return;
        }

        if(packet.autonomy_mode == AUTONOMOUS) {
            m_RcCommanding = false;
            SendLog(LogPacket::Severity::INFO, "RCControlGkcPacket is in autonomous mode, ignoring");
            return;
        }

        // If throttle, steering, and brake are zero, then the RC is not commanding
        if(packet.throttle == 0.0 && packet.steering == 0.0 && packet.brake == 0.0) {
            m_RcCommanding = packet.autonomy_mode == AutonomyMode::MANUAL;
            if(!m_RcCommanding)
                return;
        }

        if(packet.autonomy_mode == AutonomyMode::AUTONOMOUS_OVERRIDE || 
        packet.autonomy_mode == AutonomyMode::MANUAL) {
            m_RcCommanding = true;
            m_LastRcCommand = std::chrono::steady_clock::now();
        }

        SendLog(LogPacket::Severity::INFO, 
            "RCControlGkcPacket received: throttle: " + std::to_string((int)(packet.throttle * 100)) + "%, " +
            "steering: " + std::to_string((int)(packet.steering * 100)) + "%, " +
            "brake: " + std::to_string((int)(packet.brake * 100)) + "%, " +
            "autonomy_mode: " + std::to_string(packet.autonomy_mode) + ", " +
            "is_active: " + std::to_string(packet.is_active));

        float throttleSpeed = 0.0;

        if(packet.throttle > 0.0)
            throttleSpeed = packet.throttle * RC_MAX_SPEED_FORWARD;
        else if(packet.throttle < 0.0)
            throttleSpeed = packet.throttle * RC_MAX_SPEED_REVERSE; 

        SetActuationValues(throttleSpeed, packet.steering, packet.brake);

        if(packet.autonomy_mode == AutonomyMode::AUTONOMOUS || 
           packet.autonomy_mode == AutonomyMode::AUTONOMOUS_OVERRIDE)
            if((m_LastRcCommand - std::chrono::steady_clock::now()) > 
            std::chrono::milliseconds(RC_TAKEOVER_INTERVAL_MS))
                m_RcCommanding = false;
    }

    // GkcStateMachine API IMPLEMENTATION
    // TODO: Implement on_initialize
    StateTransitionResult Controller::OnInitialize(const GkcLifecycle& lastState) {
        SendLog(LogPacket::Severity::INFO, "Controller initializing");
        m_Watchdog.Arm();
        SetActuationValues(0.0, 0.0, EMERGENCY_BRAKE_PRESSURE);
        return StateTransitionResult::SUCCESS;
    }

    // TODO: Implement on_deactivate
    StateTransitionResult Controller::OnDeactivate(const GkcLifecycle& lastState) {
        SendLog(LogPacket::Severity::INFO, "Controller deactivating");
        m_ThrottleVescDisable = 1;
        m_SteeringVescDisable = 1;
        return StateTransitionResult::SUCCESS;
    }

    // TODO: Implement on_activate
    StateTransitionResult Controller::OnActivate(const GkcLifecycle& lastState) {
        SendLog(LogPacket::Severity::INFO, "Controller activating");
        m_ThrottleVescDisable = 0;
        m_SteeringVescDisable = 0;
        return StateTransitionResult::SUCCESS;
    }

    // TODO: Implement on_emergency_stop
    StateTransitionResult Controller::OnEmergencyStop(const GkcLifecycle& lastState) {
        SendLog(LogPacket::Severity::INFO, "Controller emergency stopping");
        SetActuationValues(0.0, 0.0, EMERGENCY_BRAKE_PRESSURE);
        m_ThrottleVescDisable = 1;
        m_SteeringVescDisable = 1;
        m_EmergencyActive = true;
        return StateTransitionResult::SUCCESS;
    }

    // TODO: Implement on_reinitialize
    StateTransitionResult Controller::OnReinitialize(const GkcLifecycle& lastState) {
        SendLog(LogPacket::Severity::INFO, "Controller reinitializing");
        SetActuationValues(0.0, 0.0, EMERGENCY_BRAKE_PRESSURE);
        m_ThrottleVescDisable = 0;
        m_SteeringVescDisable = 0;
        m_EmergencyActive = false;
        return StateTransitionResult::SUCCESS;
    }

    void Controller::SetActuationValues(float throttle, float steering, float brake) {
        if(GetState() != GkcLifecycle::Active) {
            SendLog(LogPacket::Severity::INFO, "Controller is not active, ignoring SetActuationValues");
            m_Actuation.FullRelRevCurrentBrake();
            m_Actuation.SetBrakeCmd(brake);
            m_Actuation.SetSteeringCmd(0.0);
            return;
        }
        m_Actuation.SetSteeringCmd(steering);
        m_Actuation.SetThrottleCmd(throttle);
        m_Actuation.SetBrakeCmd(brake);
    }

} // namespace tritonai::gkc