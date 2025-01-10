package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.*

class Arm(val io: ArmIO) {
    val inputs = ArmIO.ArmIOInputs()


    var armTargetVoltage: ElectricalPotential = 0.0.volts
    var armTargetPosition: Angle = 0.0.degrees

    var currentState = ArmState.UNINITIALIZED
    var currentRequest: Request.ArmRequest = Request.ArmRequest.Zero()
        set(value) {
            when (value) {
                is Request.ArmRequest.OpenLoop -> {
                    armTargetVoltage = value.armVoltage
                }

                is Request.ArmRequest.CloseLoop -> {
                    armTargetPosition = value.armPosition
                }

                else -> {}
            }
            field = value
        }


    // Arm Tunable Values
    private var armKP = LoggedTunableValue("Arm/armKP", Pair({it.inVoltsPerDegree}, {it.volts.perDegree}))
    private var armKI = LoggedTunableValue("Arm/armKI", Pair({it.inVoltsPerDegreeSeconds}, {it.volts.perDegreeSeconds}))
    private var armKD = LoggedTunableValue("Arm/armKD", Pair({it.inVoltsPerDegreePerSecond}, {it.volts.perDegreePerSecond}))

    init {
        // Initilize Arm Tunable Values
        armKP.initDefault(ArmConstants.ARM_KP)
        armKI.initDefault(ArmConstants.ARM_KI)
        armKD.initDefault(ArmConstants.ARM_KD)
    }

    fun periodic() {
        io.updateInputs(inputs)
        CustomLogger.processInputs("Arm", inputs)
        CustomLogger.recordOutput("Arm/currentState", currentState.toString())

        // Configure Tuneable Values
        if (armKP.hasChanged() || armKI.hasChanged() || armKD.hasChanged()) {
            io.configurePID(armKP.get(), armKI.get(), armKD.get())
        }

        var nextState = currentState
        CustomLogger.recordOutput("Arm/nextState", nextState.toString())

        when (currentState) {
            ArmState.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }

            ArmState.OPEN_LOOP -> {
                io.setArmVoltage(armTargetVoltage)
                nextState = fromRequestToState(currentRequest)
            }

            ArmState.CLOSE_LOOP -> {
                io.setArmPosition(armTargetPosition)
                nextState = fromRequestToState(currentRequest)
            }

            ArmState.ZERO -> {
                io.zeroEncoder()
                currentRequest = Request.ArmRequest.OpenLoop(0.volts)
                nextState = fromRequestToState(currentRequest)
            }
        }

        currentState = nextState
    }

    companion object {
        enum class ArmState {
            UNINITIALIZED,
            OPEN_LOOP,
            CLOSE_LOOP,
            ZERO,
        }

        // Translates Current Request to a State
        fun fromRequestToState(request: Request.ArmRequest): ArmState {
            return when (request) {
                is Request.ArmRequest.OpenLoop -> ArmState.OPEN_LOOP
                is Request.ArmRequest.CloseLoop -> ArmState.CLOSE_LOOP
                is Request.ArmRequest.Zero -> ArmState.ZERO
            }
        }
    }

}