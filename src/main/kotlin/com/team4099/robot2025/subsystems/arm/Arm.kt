package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inAmperes

class Arm(private val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  private var armTargetCurrent: Current = 0.0.amps
  var isHomed = false
  var isAtTargetedPosition: Boolean = false

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime

  private var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.TorqueControl(0.0.amps)
    set(value) {
      when (value) {
        is Request.ArmRequest.TorqueControl -> {
          armTargetCurrent = value.armCurrent
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
    CustomLogger.recordOutput("Arm/armTargetCurrent", armTargetCurrent.inAmperes)

    isAtTargetedPosition =
      !inputs.isSimulating &&
      inputs.armStatorCurrent >= ArmConstants.STALL_CURRENT &&
      (Clock.fpgaTime - lastHomingStatorCurrentTripTime) >= ArmConstants.STALL_TIME_THRESHOLD

    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.TORQUE_CURRENT -> {
        if (!isAtTargetedPosition) {
          io.setArmCurrent(armTargetCurrent)
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
        is Request.ArmRequest.TorqueControl -> ArmState.TORQUE_CURRENT
      }
    }
  }
}
