#include <control/FlightStateMachine.hpp>

namespace drone::control {

bool FlightStateMachine::transitionTo(FlightState newState) {
    if (isValidTransition(currentState_, newState)) {
        currentState_ = newState;
        return true;
    }
    return false; // Transición ilegal (ej: de IDLE a FLYING directamente)
}

bool FlightStateMachine::isValidTransition(FlightState from, FlightState to) {
    // Definimos las reglas de seguridad
    switch (from) {
        case FlightState::IDLE:
            return (to == FlightState::STANDBY || to == FlightState::FAILSAFE);
            
        case FlightState::STANDBY:
            return (to == FlightState::ARMING || to == FlightState::FAILSAFE);
            
        case FlightState::ARMING:
            return (to == FlightState::FLYING || to == FlightState::STANDBY || to == FlightState::FAILSAFE);
            
        case FlightState::FLYING:
            return (to == FlightState::FAILSAFE || to == FlightState::EMERGENCY_STOP || to == FlightState::STANDBY);
            
        case FlightState::FAILSAFE:
            // Desde failsafe normalmente solo puedes volver a IDLE tras un reset
            return (to == FlightState::IDLE);
            
        case FlightState::EMERGENCY_STOP:
            return (to == FlightState::IDLE);

        default:
            return false;
    }
}

std::string_view FlightStateMachine::getStateName() const {
    switch (currentState_) {
        case FlightState::IDLE:           return "IDLE";
        case FlightState::STANDBY:        return "STANDBY";
        case FlightState::ARMING:         return "ARMING";
        case FlightState::FLYING:         return "FLYING";
        case FlightState::FAILSAFE:       return "FAILSAFE";
        case FlightState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                          return "UNKNOWN";
    }
}

} // namespace drone::control