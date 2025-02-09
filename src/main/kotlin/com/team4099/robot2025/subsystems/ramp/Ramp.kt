package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.RampConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.math.filter.Debouncer
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts

class Ramp(val io: RampIO) {
  val inputs = RampIO.RampIOInputs()

  var currentState = RampState.UNINITIALIZED
  var currentRequest: Request.RampRequest = Request.RampRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.RampRequest.OpenLoop -> {
          rampTargetVoltage = value.voltage
        }
        else -> {}
      }
      field = value
    }

  var lastRollerRunTime = Clock.fpgaTime

  var hasCoral = false

  var debounceFilter = Debouncer(RampConstants.BEAM_BREAK_FILTER_TIME.inSeconds)

  var rampTargetVoltage: ElectricalPotential = 0.0.volts

  fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Ramp", inputs)
    CustomLogger.recordOutput("Ramp/currentState", currentState.toString())

    hasCoral = debounceFilter.calculate(inputs.beamBroken)

    var nextState = currentState
    CustomLogger.recordOutput("Ramp/nextState", nextState.toString())

    when (currentState) {
      RampState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      RampState.OPEN_LOOP -> {
        io.setVoltage(rampTargetVoltage)
        nextState = fromRequestToState(currentRequest)
        lastRollerRunTime = Clock.fpgaTime
      }
    }

    currentState = nextState
  }

  companion object {
    enum class RampState {
      UNINITIALIZED,
      OPEN_LOOP,
    }

    fun fromRequestToState(request: Request.RampRequest): RampState {
      return RampState.OPEN_LOOP
    }
  }
}
