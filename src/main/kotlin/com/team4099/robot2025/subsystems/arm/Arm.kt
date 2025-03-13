package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Arm(private val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  private var armTargetVoltage: ElectricalPotential = 0.0.volts
  var isHomed = false
  var isAtTargetedPosition: Boolean = false

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.OpenLoop(0.volts)
    set(value) {
      when (value) {
        is Request.ArmRequest.OpenLoop -> {
          armTargetVoltage = value.armVoltage
        }
        else -> {}
      }
      field = value
    }

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Arm", inputs)
    CustomLogger.recordOutput("Arm/currentState", currentState.toString())

    var nextState = currentState
    CustomLogger.recordOutput("Arm/nextState", nextState.toString())
    CustomLogger.recordOutput("Arm/armTargetVoltage", armTargetVoltage.inVolts)

    isAtTargetedPosition =
      !inputs.isSimulating &&
      inputs.armStatorCurrent >= ArmConstants.HOMING_STALL_CURRENT &&
      (Clock.fpgaTime - lastHomingStatorCurrentTripTime) >=
      ArmConstants.HOMING_STALL_TIME_THRESHOLD

    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.TORQUE_CURRENT -> {
        if (!isAtTargetedPosition) {
          io.setArmCurrent(armTargetVoltage)
          nextState = fromRequestToState(currentRequest)
        }
      }
      ArmState.HOME -> {
        if (inputs.armStatorCurrent < ArmConstants.HOMING_STALL_CURRENT) {
          lastHomingStatorCurrentTripTime = Clock.fpgaTime
        }

        if (!inputs.isSimulating &&
          (
            !isHomed &&
              inputs.armStatorCurrent < ArmConstants.HOMING_STALL_CURRENT &&
              (Clock.fpgaTime - lastHomingStatorCurrentTripTime) <
              ArmConstants.HOMING_STALL_TIME_THRESHOLD
            )
        ) {
          io.setArmCurrent(ArmConstants.HOMING_VOLTAGE)
        } else {
          isHomed = true
        }

        if (isHomed) {
          nextState = fromRequestToState(currentRequest)
        }
      }
    }

    currentState = nextState
  }

  companion object {
    enum class ArmState {
      UNINITIALIZED,
      TORQUE_CURRENT,
      HOME
    }

    // Translates Current Request to a State
    fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        is Request.ArmRequest.OpenLoop -> ArmState.OPEN_LOOP
      }
    }
  }
}
