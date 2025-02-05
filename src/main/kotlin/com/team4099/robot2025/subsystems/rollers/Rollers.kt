package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

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
      field = value
    }

  var lastRollerRunTime = Clock.fpgaTime

  val hasCoral: Boolean
    get() {
      return inputs.rollerVelocity.absoluteValue <= RollersConstants.CORAL_VELOCITY_THRESHOLD &&
        inputs.rollerStatorCurrent > RollersConstants.CORAL_CURRENT_THRESHOLD &&
        inputs.rollerAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastRollerRunTime) >= RollersConstants.CORAL_DETECTION_TIME_THRESHOLD ||
        inputs.isSimulating
    }

  val hasAlgae: Boolean
    get() {
      return inputs.rollerVelocity.absoluteValue <= RollersConstants.ALGAE_VELOCITY_THRESHOLD &&
        inputs.rollerStatorCurrent > RollersConstants.ALGAE_CURRENT_THRESHOLD &&
        inputs.rollerAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastRollerRunTime) >= RollersConstants.ALGAE_DETECTION_TIME_THRESHOLD ||
        inputs.isSimulating
    }

  var rollersTargetVoltage: ElectricalPotential = 0.0.volts

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Rollers", inputs)
    CustomLogger.recordOutput("Rollers/currentState", currentState.toString())

    var nextState = currentState
    CustomLogger.recordOutput("Rollers/nextState", nextState.toString())

    when (currentState) {
      RollersState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      RollersState.OPEN_LOOP -> {
        io.setRollerVoltage(rollersTargetVoltage)
        nextState = fromRequestToState(currentRequest)
        lastRollerRunTime = Clock.fpgaTime
      }
    }

    currentState = nextState
  }

  companion object {
    enum class RollersState {
      UNINITIALIZED,
      OPEN_LOOP,
    }

    fun fromRequestToState(request: Request.RollersRequest): RollersState {
      return RollersState.OPEN_LOOP
    }
  }
}
