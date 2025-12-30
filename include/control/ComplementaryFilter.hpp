#ifndef COMPLEMENTARY_FILTER_HPP
#define COMPLEMENTARY_FILTER_HPP

#include <chrono>
#include <cmath>

namespace drone::control {

/**
 * @brief Filtro Complementario para estimación de actitud.
 * @tparam T Tipo numérico (float o double).
 */
template <typename T>
class ComplementaryFilter {
public:
    /**
     * @param alpha Factor de confianza (0.95 - 0.99). 
     * Valores altos confían más en el giroscopio.
     */
    explicit ComplementaryFilter(T alpha) 
        : alpha_(alpha), angle_(0) {}

    /**
     * @brief Actualiza la estimación del ángulo.
     * @param accelAngle Ángulo calculado por el acelerómetro (atan2).
     * @param gyroRate Velocidad angular del giroscopio (grados o radianes/seg).
     * @param dt Delta tiempo.
     */
    T update(T accelAngle, T gyroRate, std::chrono::duration<T> dt) {
        T seconds = dt.count();
        
        // Fusión: 
        // Nuevo Ángulo = Alpha * (Ángulo previo + Integración Giro) + (1 - Alpha) * (Acel)
        angle_ = alpha_ * (angle_ + gyroRate * seconds) + (1 - alpha_) * accelAngle;
        
        return angle_;
    }

    void setAngle(T angle) { angle_ = angle; }
    T getAngle() const { return angle_; }

private:
    T alpha_;
    T angle_;
};

} // namespace drone::control

#endif