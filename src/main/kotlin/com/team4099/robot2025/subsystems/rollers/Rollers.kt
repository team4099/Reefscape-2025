package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.math.filter.Debouncer
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Rollers(val io: RollersIO) {
  val inputs = RollersIO.RollersIOInputs()

  var currentState = RollersState.UNINITIALIZED
  var currentRequest: Request.RollersRequest = Request.RollersRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.RollersRequest.OpenLoop -> {
          rollersTargetVoltage = value.voltage
        }
        else -> {}
      }
      field = value
    }

  var lastRollerVoltageTarget = 0.0.volts
  var lastRollerRunTime = Clock.fpgaTime

  var debounceFilter = Debouncer(RollersConstants.BEAM_BREAK_FILTER_TIME.inSeconds)

  val hasCoralVertical: Boolean
    get() {
      return inputs.rollerStatorCurrent > RollersConstants.CORAL_CURRENT_THRESHOLD &&
        !inputs.rollerAppliedVoltage.epsilonEquals(
          RollersConstants
            .SCORE_CORAL_VOLTAGE
        ) && // make sure score & intake are not the same voltage
        (Clock.fpgaTime - lastRollerRunTime) >= RollersConstants.CORAL_DETECTION_TIME_THRESHOLD
    }

  val hasCoralHorizontal: Boolean
    get() {
      return inputs.rollerStatorCurrent > RollersConstants.CORAL_HORIZONTAL_CURRENT_THRESHOLD &&
        inputs.rollerAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastRollerRunTime) >=
        RollersConstants.CORAL_HORIZONTAL_DETECTION_TIME_THRESHOLD
    }

  val hasAlgae: Boolean
    get() {
      return inputs.rollerStatorCurrent > RollersConstants.ALGAE_CURRENT_THRESHOLD &&
        inputs.rollerAppliedVoltage.sign < 0 &&
        (Clock.fpgaTime - lastRollerRunTime) >= RollersConstants.ALGAE_DETECTION_TIME_THRESHOLD
    }

  var rollersTargetVoltage: ElectricalPotential = 0.0.volts

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Rollers", inputs)
    CustomLogger.recordOutput("Rollers/currentState", currentState.toString())

    //    hasCoralVertical = debounceFilter.calculate(inputs.beamBroken)

    var nextState = currentState
    CustomLogger.recordOutput("Rollers/nextState", nextState.toString())
    CustomLogger.recordOutput("Rollers/hasCoralVertical", hasCoralVertical)
    CustomLogger.recordOutput("Rollers/targetVoltage", rollersTargetVoltage.inVolts)

    when (currentState) {
      RollersState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      RollersState.OPEN_LOOP -> {
        io.setVoltage(rollersTargetVoltage)
        nextState = fromRequestToState(currentRequest)

        if (lastRollerVoltageTarget != rollersTargetVoltage) {
          if (rollersTargetVoltage != RollersConstants.INTAKE_CORAL_VOLTAGE_SLOW) {
            lastRollerVoltageTarget = rollersTargetVoltage
            lastRollerRunTime = Clock.fpgaTime
          }
        }
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
