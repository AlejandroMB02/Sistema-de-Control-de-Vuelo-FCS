# UAV Flight Control Core (FCS)

[![C++ Standard](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)
[![Build Status](https://img.shields.io/badge/Build-CMake-green.svg)](#)
[![Unit Tests](https://img.shields.io/badge/Tests-GoogleTest-red.svg)](#)

Este repositorio contiene el núcleo de un **Sistema de Control de Vuelo (FCS)** para drones multirrotores, desarrollado con un enfoque estricto en la arquitectura de software profesional, robustez y mantenibilidad.



## 🚀 Características Principales

* **Controlador PID de Alto Rendimiento:** Algoritmo genérico que incluye protección **Anti-Windup** y saturación de salida para evitar la inestabilidad del sistema.
* **Filtro Complementario de Actitud:** Implementación para la fusión de datos de sensores, optimizando la estimación del ángulo mediante la combinación de acelerómetro y giroscopio.
* **Máquina de Estados de Vuelo (FSM):** Lógica de control que garantiza la seguridad operativa mediante validación de transiciones entre estados como `IDLE`, `ARMING` y `FLYING`.
* **Diseño Basado en Plantillas (Templates):** Los controladores están desacoplados del tipo de dato, permitiendo el uso de `float` o `double` según la capacidad del hardware.

## 🛠️ Stack Tecnológico

El proyecto utiliza herramientas estándar de la industria aeroespacial y de robótica:

* **Lenguaje:** C++20 (Uso de `std::chrono` para gestión de tiempo precisa y `std::clamp` para seguridad de señales).
* **Sistema de Construcción:** CMake 3.16+ con soporte para exportación de comandos de compilación para herramientas de análisis estático.
* **Unit Testing:** GoogleTest para la verificación exhaustiva de la lógica de control y seguridad.
* **Documentación:** Doxygen para la generación automática de manuales técnicos de la API.

## 📂 Arquitectura del Software

El sistema se divide en módulos independientes para facilitar la integración y el testeo:

### 1. Control de Actitud e Integración Temporal
[cite_start]Se utiliza el tipo `std::chrono::duration` para todos los cálculos físicos, eliminando errores de unidades y garantizando que las constantes $K_p$, $K_i$ y $K_d$ operen correctamente independientemente de la frecuencia del bucle[cite: 1].

### 2. Lógica de Seguridad (FSM)
La máquina de estados actúa como un "guardián" del sistema, impidiendo transiciones críticas (como pasar de `IDLE` directamente a `FLYING`) y gestionando estados de error como `FAILSAFE` o `EMERGENCY_STOP`.



## 🧪 Calidad y Testing

La fiabilidad es el pilar de este proyecto. La suite de pruebas incluye:

* **Pruebas de PID:** Verificación de la acumulación integral, respuesta derivativa ante cambios bruscos y robustez ante deltas de tiempo nulos ($dt = 0$).
* **Validación de Transiciones:** Garantía de que el dron solo puede armarse si ha pasado previamente por el estado de `STANDBY`.

### Ejecución de Tests:
```bash
git submodule add -f https://github.com/google/googletest.git extern/googletest
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j$(nproc)
ctest
```
### Generación de documentación:
El proyecto está totalmente documentado siguiendo el estándar Doxygen.
```bash
cd build
make doc_doxygen
```
### Contacto
Este proyecto forma parte de mi portfolio de ingeniería. Si tienes preguntas sobre las decisiones de diseño o la implementación de algoritmos de control, no dudes en contactarme.
