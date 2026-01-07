#ifndef FLIGHT_STATE_MACHINE_HPP
#define FLIGHT_STATE_MACHINE_HPP

#include <string_view>

namespace drone::control {

/**
 * @brief Estados del sistema de vuelo.
 */
enum class FlightState {
    IDLE,           // Sistema encendido pero esperando inicialización
    STANDBY,        // Sensores listos, esperando comando de armado
    ARMING,         // Comprobaciones de seguridad y arranque de motores
    FLYING,         // PID y Control activo
    FAILSAFE,       // Error crítico detectado (pérdida de señal, batería baja)
    EMERGENCY_STOP  // Parada inmediata de motores
};

class FlightStateMachine {
public:
    FlightStateMachine() = default;

    /**
     * @brief Intenta cambiar al siguiente estado validando la transición.
     * @return true si la transición fue válida.
     */
    bool transitionTo(FlightState newState);

    /**
     * @brief Obtiene el estado actual.
     */
    FlightState getCurrentState() const { return currentState_; }

    /**
     * @brief Devuelve el nombre del estado en texto (útil para telemetría).
     */
    std::string_view getStateName() const;

private:
    FlightState currentState_{FlightState::IDLE};
    
    // Método interno para validar si el salto de estado es legal
    bool isValidTransition(FlightState from, FlightState to);
};

} // namespace drone::control

#endif