package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

class Arm(private val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  private var armTargetPosition: Angle = 0.0.degrees
  val isAtTargetedPosition: Boolean
    get() {
      return (inputs.armPosition - armTargetPosition).absoluteValue <= ArmConstants.ARM_TOLERANCE
    }
  var currentPosition: ArmPosition = ArmPosition.RETRACTED

  private var lastHomingStatorCurrentTripTime = Clock.fpgaTime
  private var lastRequestedState: ArmState = ArmState.UNINITIALIZED

  private var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.Zero()
    set (value) {
      if (value is Request.ArmRequest.ClosedLoop) {
        armTargetPosition = value.position
      }

      field = value
    }

  init {
    io.configurePID(
      ArmConstants.PID.REAL_KP,
      ArmConstants.PID.REAL_KI,
      ArmConstants.PID.REAL_KD
    )
  }
  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Arm", inputs)
    CustomLogger.recordOutput("Arm/currentState", currentState.toString())

    var nextState = currentState
    CustomLogger.recordOutput("Arm/nextState", nextState.toString())
    CustomLogger.recordOutput("Arm/armTargetPosition", armTargetPosition.inDegrees)

    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.ZERO -> {
        io.zeroEncoder()
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.CLOSED_LOOP -> {
        io.setArmPosition(armTargetPosition)
        nextState = fromRequestToState(currentRequest)
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
      ZERO,
      CLOSED_LOOP
    }

    // Translates Current Request to a State
    fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        // is Request.ArmRequest.TorqueControl -> ArmState.TORQUE_CURRENT
        is Request.ArmRequest.Zero -> ArmState.ZERO
        is Request.ArmRequest.ClosedLoop -> ArmState.CLOSED_LOOP
      }
    }
  }
}
