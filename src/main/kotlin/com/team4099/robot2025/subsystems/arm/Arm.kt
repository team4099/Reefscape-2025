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
  val isAtTargetedPosition: Boolean
    get() {
      return currentState == lastRequestedState
    }
  var currentPosition: ArmPosition = ArmPosition.RETRACTED

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime
  private var lastRequestedState: ArmState = ArmState.UNINITIALIZED

  private var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.Retract()

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Arm", inputs)
    CustomLogger.recordOutput("Arm/currentState", currentState.toString())

    var nextState = currentState
    CustomLogger.recordOutput("Arm/nextState", nextState.toString())
    CustomLogger.recordOutput("Arm/armTargetCurrent", armTargetCurrent.inAmperes)

    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.RETRACTED -> {
        if (currentPosition != ArmPosition.RETRACTED) {
          io.setArmCurrent(ArmTunableValues.ArmCurrents.retractCurrent.get())
        }
        if (inputs.armStatorCurrent > ArmConstants.STATOR_CURRENT_LIMIT) {
          currentPosition = ArmPosition.RETRACTED
          nextState = fromRequestToState(currentRequest)
        }
      }
      ArmState.EXTENDED -> {
        if (currentPosition != ArmPosition.EXTENDED) {
          io.setArmCurrent(ArmTunableValues.ArmCurrents.extendCurrent.get())
        }
        if (inputs.armStatorCurrent < -ArmConstants.STATOR_CURRENT_LIMIT) {
          currentPosition = ArmPosition.EXTENDED
          nextState = fromRequestToState(currentRequest)
        }
      }
    }

    currentState = nextState
  }

  companion object {
    enum class ArmPosition {
      EXTENDED,
      RETRACTED,
    }

    enum class ArmState {
      UNINITIALIZED,
      EXTENDED,
      RETRACTED
    }

    // Translates Current Request to a State
    fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        // is Request.ArmRequest.TorqueControl -> ArmState.TORQUE_CURRENT
        is Request.ArmRequest.Extend -> ArmState.EXTENDED
        is Request.ArmRequest.Retract -> ArmState.RETRACTED
      }
    }
  }
}
