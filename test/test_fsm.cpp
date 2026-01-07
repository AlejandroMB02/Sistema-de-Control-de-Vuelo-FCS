#include <gtest/gtest.h>
#include <control/FlightStateMachine.hpp>

using drone::control::FlightStateMachine;
using drone::control::FlightState;

class FsmTest : public ::testing::Test {
protected:
    FlightStateMachine fsm;
};

// Test 1: El estado inicial debe ser IDLE
TEST_F(FsmTest, InitialStateIsIdle) {
    EXPECT_EQ(fsm.getCurrentState(), FlightState::IDLE);
}

// Test 2: Camino feliz (Secuencia lógica de despegue)
TEST_F(FsmTest, NormalTakeoffSequence) {
    EXPECT_TRUE(fsm.transitionTo(FlightState::STANDBY));
    EXPECT_TRUE(fsm.transitionTo(FlightState::ARMING));
    EXPECT_TRUE(fsm.transitionTo(FlightState::FLYING));
    EXPECT_EQ(fsm.getCurrentState(), FlightState::FLYING);
}

// Test 3: Bloqueo de seguridad (No saltar pasos)
TEST_F(FsmTest, PreventIllegalJumpToFlying) {
    // Intentar pasar de IDLE a FLYING directamente debe fallar
    EXPECT_FALSE(fsm.transitionTo(FlightState::FLYING));
    EXPECT_EQ(fsm.getCurrentState(), FlightState::IDLE);
}

// Test 4: Parada de emergencia desde cualquier estado crítico
TEST_F(FsmTest, EmergencyStopFromFlying) {
    // Llegamos a vuelo
    fsm.transitionTo(FlightState::STANDBY);
    fsm.transitionTo(FlightState::ARMING);
    fsm.transitionTo(FlightState::FLYING);

    // Activamos parada de emergencia
    EXPECT_TRUE(fsm.transitionTo(FlightState::EMERGENCY_STOP));
    EXPECT_EQ(fsm.getCurrentState(), FlightState::EMERGENCY_STOP);
}

// Test 5: Recuperación de Failsafe
TEST_F(FsmTest, FailsafeRecoveryRequiresReset) {
    fsm.transitionTo(FlightState::STANDBY);
    fsm.transitionTo(FlightState::FAILSAFE);
    
    // Desde FAILSAFE no deberíamos poder ir a ARMING sin pasar por IDLE (Reset)
    EXPECT_FALSE(fsm.transitionTo(FlightState::ARMING));
    EXPECT_TRUE(fsm.transitionTo(FlightState::IDLE));
}