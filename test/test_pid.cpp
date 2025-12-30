#include <gtest/gtest.h>
#include <control/PidController.hpp>
#include <chrono>

using namespace std::chrono_literals;
using drone::control::PidController;

class PidTest : public ::testing::Test {
protected:
    // Configuración base para pruebas aisladas
    PidController<float>::Config baseConfig{
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
        .minOutput = -100.0f, .maxOutput = 100.0f
    };
};

// 1. Test de Acción Proporcional
TEST_F(PidTest, ProportionalActionOnly) {
    baseConfig.kp = 1.5f;
    PidController<float> pid(baseConfig);

    // Error = 10 -> Salida = 10 * 1.5 = 15
    auto output = pid.calculate(10.0f, 0.0f, 10ms);
    EXPECT_NEAR(output, 15.0f, 0.001f);
}

// 2. Test de Acción Integral (Acumulación)
TEST_F(PidTest, IntegralActionAccumulation) {
    baseConfig.ki = 1.0f;
    PidController<float> pid(baseConfig);

    // Primer paso: Error = 10, dt = 1s -> Integral = 10
    pid.calculate(10.0f, 0.0f, 1s);
    
    // Segundo paso: Error = 10, dt = 1s -> Integral acumulada = 20
    float output = pid.calculate(10.0f, 0.0f, 1s);
    EXPECT_NEAR(output, 20.0f, 0.001f);
}

// 3. Test de Acción Derivativa (Reacción al cambio)
TEST_F(PidTest, DerivativeActionReaction) {
    baseConfig.kd = 0.1f;
    PidController<float> pid(baseConfig);

    // Inicializamos el PID con el primer valor
    pid.calculate(10.0f, 0.0f, 10ms); 

    // Cambiamos bruscamente el valor medido (el dron se inclina rápido)
    // Error previo = 10, Nuevo error = 5 -> Delta Error = -5
    // Salida D = Kd * (de/dt) = 0.1 * (-5 / 0.01) = -50
    float output = pid.calculate(10.0f, 5.0f, 10ms);
    EXPECT_NEAR(output, -50.0f, 0.001f);
}

// 4. Test de Anti-Windup (Saturación de la Integral)
TEST_F(PidTest, IntegralSaturatesAtLimits) {
    baseConfig.ki = 100.0f; // Ki muy alto para saturar rápido
    PidController<float> pid(baseConfig);

    // Forzamos error masivo durante mucho tiempo
    float output = 0;
    for(int i = 0; i < 10; ++i) {
        output = pid.calculate(100.0f, 0.0f, 1s);
    }

    EXPECT_EQ(output, baseConfig.maxOutput);
}

// 5. Test de Reset (Limpieza de estado)
TEST_F(PidTest, ResetClearsState) {
    baseConfig.kp = 1.0f; baseConfig.ki = 1.0f;
    PidController<float> pid(baseConfig);

    // Generamos histórico
    pid.calculate(10.0f, 0.0f, 1s);
    pid.reset();

    // Tras el reset, con error 0 la salida DEBE ser 0 (la integral murió)
    auto output = pid.calculate(0.0f, 0.0f, 1ms);
    EXPECT_FLOAT_EQ(output, 0.0f);
}

// 6. Test de Robustez: Delta Tiempo Cero
TEST_F(PidTest, HandlesZeroDtGracefully) {
    baseConfig.kd = 1.0f;
    PidController<float> pid(baseConfig);

    auto output = pid.calculate(10.0f, 0.0f, 0ms);
    
    // Un valor NaN nunca es igual a sí mismo. 
    EXPECT_TRUE(output == output) << "La salida del PID es NaN (posible división por cero)";
}