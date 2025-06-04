/**
 * @file state_machine.cpp
 * @brief Implementation of the system lifecycle state machine
 * 
 * @copyright Copyright 2025 Triton AI
 */

#include "state_machine.hpp"
#include <iostream>

namespace tritonai::gkc {

    GkcStateMachine::GkcStateMachine() : m_State(GkcLifecycle::Uninitialized) {
        CommonChecks();
    }

    StateTransitionResult GkcStateMachine::Initialize() {
        if (m_State != GkcLifecycle::Uninitialized) {
            return StateTransitionResult::FAILURE_INVALID_TRANSITION;
        }
        
        m_State = GkcLifecycle::Initializing;
        const auto result = OnInitialize(m_State);
        
        switch (result) {
        case StateTransitionResult::SUCCESS:
            m_State = GkcLifecycle::Inactive;
            break;
        case StateTransitionResult::EMERGENCY_STOP:
            m_State = GkcLifecycle::Emergency;
            break;
        default:
            m_State = GkcLifecycle::Uninitialized;
            break;
        }
        
        CommonChecks();
        return result;
    }

    StateTransitionResult GkcStateMachine::Deactivate() {
        if (m_State != GkcLifecycle::Active) {
            return StateTransitionResult::FAILURE_INVALID_TRANSITION;
        }
        
        const auto result = OnDeactivate(m_State);
        
        switch (result) {
        case StateTransitionResult::SUCCESS:
            m_State = GkcLifecycle::Inactive;
            break;
        case StateTransitionResult::EMERGENCY_STOP:
            m_State = GkcLifecycle::Emergency;
            break;
        default:
            m_State = GkcLifecycle::Active;
            break;
        }
        
        CommonChecks();
        return result;
    }

    StateTransitionResult GkcStateMachine::Activate() {
        if (m_State != GkcLifecycle::Inactive) {
            return StateTransitionResult::FAILURE_INVALID_TRANSITION;
        }
        
        const auto result = OnActivate(m_State);
        
        switch (result) {
        case StateTransitionResult::SUCCESS:
            m_State = GkcLifecycle::Active;
            break;
        case StateTransitionResult::EMERGENCY_STOP:
            m_State = GkcLifecycle::Emergency;
            break;
        default:
            m_State = GkcLifecycle::Inactive;
            break;
        }
        
        CommonChecks();
        return result;
    }

    StateTransitionResult GkcStateMachine::EmergencyStop() {
        if (m_State == GkcLifecycle::Uninitialized) {
            return StateTransitionResult::FAILURE_INVALID_TRANSITION;
        }
        
        if (m_State != GkcLifecycle::Emergency) {
            m_State = GkcLifecycle::Emergency;
            return EmergencyStop();
        }
        
        const auto result = OnEmergencyStop(m_State);
        
        switch (result) {
        case StateTransitionResult::SUCCESS:
            m_State = GkcLifecycle::Inactive;
            break;
        case StateTransitionResult::ERROR:
            NVIC_SystemReset();
        default:
            m_State = GkcLifecycle::Emergency;
            break;
        }
        
        CommonChecks();
        return result;
    }

    StateTransitionResult GkcStateMachine::Reinitialize() {
        if (m_State != GkcLifecycle::Uninitialized) {
            return StateTransitionResult::FAILURE_INVALID_TRANSITION;
        }
        
        m_State = GkcLifecycle::Initializing;
        const auto result = OnReinitialize(m_State);
        
        switch (result) {
        case StateTransitionResult::SUCCESS:
            m_State = GkcLifecycle::Inactive;
            break;
        default:
            m_State = GkcLifecycle::Uninitialized;
            break;
        }
        
        CommonChecks();
        return result;
    }

    GkcLifecycle GkcStateMachine::GetState() const { 
        return m_State; 
    }

    void GkcStateMachine::CommonChecks() {
        if (m_State == GkcLifecycle::Active)
            m_Led = 0;
        else
            m_Led = 1;
    }

} // namespace tritonai::gkc