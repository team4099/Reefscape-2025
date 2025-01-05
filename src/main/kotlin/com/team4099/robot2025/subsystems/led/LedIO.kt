package com.team4099.robot2023.subsystems.led

import com.team4099.robot2023.config.constants.LEDConstants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.derived.ElectricalPotential

interface LedIO {
  var batteryVoltage: ElectricalPotential

  class LedIOInputs : LoggableInputs {
    var ledState = LEDConstants.CandleState.NO_NOTE.toString()

    override fun toLog(table: LogTable?) {
      table?.put("ledState", ledState.toString())
    }

    override fun fromLog(table: LogTable?) {
      table?.getString("ledState", ledState.toString())?.let { ledState = it }
    }
  }

  fun setState(newState: LEDConstants.CandleState) {}

  fun updateInputs(inputs: LedIOInputs) {}
}
