/**
 * @file state_machine.hpp
 * @brief State machine for controlling system lifecycle
 * 
 * @copyright Copyright 2025 Triton AI
 */

#pragma once

#include "mbed.h"
#include "tai_gokart_packet/gkc_packet_utils.hpp"

namespace tritonai::gkc {

    // The possible results of a state transition
    enum StateTransitionResult {
        SUCCESS = 0,
        FAILURE = 1,
        ERROR = 2,
        EMERGENCY_STOP = 3,
        FAILURE_INVALID_TRANSITION = 4
    };

    /**
     * @class GkcStateMachine
     * @brief Abstract state machine for controlling system lifecycle
     */
    class GkcStateMachine {
    public:
        GkcStateMachine();

        StateTransitionResult Initialize();
        StateTransitionResult Deactivate();
        StateTransitionResult Activate();
        StateTransitionResult EmergencyStop();
        StateTransitionResult Reinitialize();
        
        GkcLifecycle GetState() const;

    protected:
        // State transition handlers to be implemented by child classes
        virtual StateTransitionResult OnInitialize(const GkcLifecycle& lastState) = 0;
        virtual StateTransitionResult OnDeactivate(const GkcLifecycle& lastState) = 0;
        virtual StateTransitionResult OnActivate(const GkcLifecycle& lastState) = 0;
        virtual StateTransitionResult OnEmergencyStop(const GkcLifecycle& lastState) = 0;
        virtual StateTransitionResult OnReinitialize(const GkcLifecycle& lastState) = 0;

    private:
        GkcLifecycle m_State{GkcLifecycle::Uninitialized};
        void CommonChecks();
        DigitalOut m_Led{LED3, 0};
    };

} // namespace tritonai::gkc