#include "control/PidController.hpp"
#include "control/ComplementaryFilter.hpp"
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

int main() {
    // Usamos 'float' para simular el entorno del ESP32
    using DataType = float;

    // Configuración del PID para el eje de Roll (Alabeo)
    drone::control::PidController<DataType>::Config rollConfig{
        .kp = 1.5f, .ki = 0.05f, .kd = 0.1f,
        .minOutput = -100.0f, .maxOutput = 100.0f
    };

    drone::control::PidController<DataType> rollPid(rollConfig);
    drone::control::ComplementaryFilter<DataType> filter(0.98f);

    // Simulación de un bucle de control a 100Hz
    DataType currentAngle = 0.0f;
    const DataType targetAngle = 0.0f; // Queremos que el dron esté nivelado
    auto dt = 10ms; 

    for (int i = 0; i < 100; ++i) {
        // 1. Simular lectura de sensores (con ruido)
        DataType accelReading = 5.0f; // El dron está inclinado 5 grados
        DataType gyroRate = -0.1f;    // El dron se está moviendo lentamente

        // 2. Fusión de sensores
        currentAngle = filter.update(accelReading, gyroRate, dt);

        // 3. Cálculo del PID
        DataType motorCorrection = rollPid.calculate(targetAngle, currentAngle, dt);

        std::cout << "Ángulo: " << currentAngle 
                  << " | Corrección Motor: " << motorCorrection << std::endl;
        
        std::this_thread::sleep_for(dt);
    }

    return 0;
}