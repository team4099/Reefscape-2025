package com.team4099.robot2025.subsystems.led

import com.team4099.robot2025.config.constants.LEDConstants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.derived.ElectricalPotential

interface LedIO {

  class LedIOInputs : LoggableInputs {
    var ledState = LEDConstants.CandleState.NOTHING.toString()

    override fun toLog(table: LogTable?) {
      table?.put("ledState", ledState.toString())
    }

    override fun fromLog(table: LogTable?) {
      table?.get("ledState", ledState)?.let { ledState = it }
    }
  }

  fun setState(newState: LEDConstants.CandleState) {}

  fun updateInputs(inputs: LedIOInputs) {}
}
