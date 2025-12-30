#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>
#include <chrono>

namespace drone::control {

/**
 * @brief Controlador PID genérico con Anti-Windup y Saturación.
 * @tparam T Tipo numérico (float o double).
 */
template <typename T>
class PidController {
public:
    struct Config {
        T kp, ki, kd;
        T minOutput;
        T maxOutput;
    };

    explicit PidController(const Config& config) 
        : config_(config), integralTerm_(0), lastError_(0) {}

    /**
     * @brief Calcula la salida del PID.
     * @param setpoint Valor deseado.
     * @param measuredValue Valor actual del sensor.
     * @param dt Tiempo transcurrido desde el último cálculo.
     */
    T calculate(T setpoint, T measuredValue, std::chrono::duration<T> dt) {
        T error = setpoint - measuredValue;
        T seconds = dt.count();

        if (!initialized_) {
            lastError_ = error;
            initialized_ = true;
        }

        // Proporcional
        T pTerm = config_.kp * error;

        // Integral con Anti-Windup (limitación simple)
        integralTerm_ += error * seconds;
        T iTerm = config_.ki * integralTerm_;
        iTerm = std::clamp(iTerm, config_.minOutput, config_.maxOutput);

        // Derivativo (basado en el cambio del error)
        T dTerm = 0;
        if (seconds > 0) {
            dTerm = config_.kd * (error - lastError_) / seconds;
        }

        lastError_ = error;

        // Salida total limitada
        T output = pTerm + iTerm + dTerm;
        return std::clamp(output, config_.minOutput, config_.maxOutput);
    }

    void reset() {
        integralTerm_ = 0;
        lastError_ = 0;
    }

private:
    Config config_;
    T integralTerm_;
    T lastError_;
    bool initialized_{false};
};

} // namespace drone::control

#endif