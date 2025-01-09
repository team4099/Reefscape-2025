package com.team4099.robot2025.subsystems.arm

import org.littletonrobotics.junction.Logger

class Rollers(val io: RollersIO) {
    val inputs = RollersIO.RollersIOInputs()

    var currentState = RollersState.UNINITIALIZED

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers", inputs)
        Logger.recordOutput("Rollers/currentState", currentState.toString())

        var nextState = currentState
        Logger.recordOutput("Rollers/nextState", nextState.toString())

        when (currentState) {
            RollersState.UNINITIALIZED -> {

            }

            RollersState.OPEN_LOOP -> {

            }

            RollersState.CLOSED_LOOP -> {

            }

            RollersState.HOME -> {

            }
        }
    }

    companion object {
        enum class RollersState {
            UNINITIALIZED,
            OPEN_LOOP,
            CLOSED_LOOP,
            HOME,
        }
    }

}