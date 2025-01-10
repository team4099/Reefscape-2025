package com.team4099.robot2025.subsystems.arm

import com.team4099.robot2025.subsystems.rollers.RollersIO
import com.team4099.robot2025.subsystems.superstructure.Request
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

class Rollers(val io: RollersIO) {
    val inputs = RollersIO.RollersIOInputs()

    var currentState = RollersState.UNINITIALIZED
    var currentRequest: Request.RollersRequest = Request.RollersRequest.OpenLoop(0.0.volts)
        set(value) {
            when (value) {
                is Request.RollersRequest.OpenLoop -> {
                    rollersTargetVoltage = value.RollersVoltage
                }
                else -> {}
            }
        }

    var rollersTargetVoltage: ElectricalPotential = 0.0.volts

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers", inputs)
        Logger.recordOutput("Rollers/currentState", currentState.toString())

        var nextState = currentState
        Logger.recordOutput("Rollers/nextState", nextState.toString())

        when (currentState) {
            RollersState.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }

            RollersState.OPEN_LOOP -> {
                io.setRollerVoltage(rollersTargetVoltage)
                nextState = fromRequestToState(currentRequest)
            }
        }
    }

    companion object {
        enum class RollersState {
            UNINITIALIZED,
            OPEN_LOOP,
        }

        fun fromRequestToState(request: Request.RollersRequest) :RollersState {
            return  RollersState.OPEN_LOOP
        }
    }

}