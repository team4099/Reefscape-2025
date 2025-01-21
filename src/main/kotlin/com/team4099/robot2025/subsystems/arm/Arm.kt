package com.team4099.robot2025.subsystems.arm

import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKD
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKI
import com.team4099.robot2025.subsystems.arm.ArmTunableValues.armKP
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class Arm(val io: ArmIO) {
  val inputs = ArmIO.ArmIOInputs()

  var armTargetVoltage: ElectricalPotential = 0.0.volts
  var armTargetPosition: Angle = 0.0.degrees

  var isZeroed = false

  var currentState = ArmState.UNINITIALIZED
  var currentRequest: Request.ArmRequest = Request.ArmRequest.Zero()
    set(value) {
      when (value) {
        is Request.ArmRequest.OpenLoop -> {
          armTargetVoltage = value.armVoltage
        }
        is Request.ArmRequest.ClosedLoop -> {
          armTargetPosition = value.armPosition
        }
        else -> {}
      }
      field = value
    }

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentState == ArmState.CLOSED_LOOP &&
          (inputs.armPosition - armTargetPosition).absoluteValue <= ArmConstants.ARM_TOLERANCE
        ) ||
        inputs.isSimulating

  init {
    // Initialize Arm Tunable Values
    armKP.initDefault(ArmConstants.ARM_KP)
    armKI.initDefault(ArmConstants.ARM_KI)
    armKD.initDefault(ArmConstants.ARM_KD)
  }

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Arm", inputs)
    CustomLogger.recordOutput("Arm/currentState", currentState.toString())

    // Configure Tunable Values
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
      ArmState.CLOSED_LOOP -> {
        io.setArmPosition(armTargetPosition)
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.ZERO -> {
        io.zeroEncoder()
        currentRequest = Request.ArmRequest.OpenLoop(0.volts)
        isZeroed = true
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  companion object {
    enum class ArmState {
      UNINITIALIZED,
      OPEN_LOOP,
      CLOSED_LOOP,
      ZERO,
    }

    // Translates Current Request to a State
    fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        is Request.ArmRequest.OpenLoop -> ArmState.OPEN_LOOP
        is Request.ArmRequest.ClosedLoop -> ArmState.CLOSED_LOOP
        is Request.ArmRequest.Zero -> ArmState.ZERO
      }
    }
  }
}
