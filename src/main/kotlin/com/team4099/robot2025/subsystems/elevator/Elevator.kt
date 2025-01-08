package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

class Elevator(val io: ElevatorIO) {
    val inputs = ElevatorIO.ElevatorInputs()

    var currentState: ElevatorState = ElevatorState.UNINITIALIZED

    var targetVoltage: ElectricalPotential = 0.0.volts

    val upperLimitReached: Boolean
        get() = inputs.elevatorPosition >= ElevatorConstants.UPWARDS_EXTENSION_LIMIT

    val lowerLimitReached: Boolean
        get() = inputs.elevatorPosition <= ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT

    var isHomed = false

    var isHomed = false

    fun periodic() {
        io.updateInputs(inputs)

        CustomLogger.processInputs("Elevator", inputs)

        var nextState = currentState

        when (currentState) {
            ElevatorState.UNINITIALIZED -> {
                nextState = ElevatorState.HOME
            }

            ElevatorState.HOME -> {
                //record the time since we last homed
                if(inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT){
                    homingCurrentSpikeStartTime = Clock.fpgaTime
                }

                // make sure we're not already homed and we are within the time to home
                if (!isHomed && inputs.leaderStatorCurrent < ElevatorConstants.HOMING_STALL_CURRENT &&
                    (Clock.fpgaTime - homingCurrentSpikeStartTime) < ElevatorConstants.HOMING_STALL_TIME_THRESHOLD) {
                    setVoltage(ElevatorConstants.HOMING_APPLIED_VOLTAGE)
                } else {
                    //if were already homed zero the encoder and set the variable to be true
                    io.zeroEncoder()
                    isHomed = true
                }

                if (isHomed) {
                    //TODO: fromrequesttostate
                    nextState = ElevatorState.OPEN_LOOP
                }
            }

            ElevatorState.OPEN_LOOP -> {
                setVoltage(targetVoltage)
            }

            ElevatorState.CLOSED_LOOP -> {

            }
        }

        currentState = nextState
    }

    fun setVoltage(targetVoltage: ElectricalPotential) {
        if ((upperLimitReached && targetVoltage > 0.0.volts) || (lowerLimitReached && targetVoltage < 0.0.volts)) {
            io.setVoltage(0.0.volts)
        } else {
            io.setVoltage(targetVoltage)
        }
    }


    companion object {
        enum class ElevatorState {
            UNINITIALIZED,
            OPEN_LOOP,
            CLOSED_LOOP,
            HOME
        }
    }
}