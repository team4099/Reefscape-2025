package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.RampConstants
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object RampTunableValues {
  val idleVoltage =
    LoggedTunableValue(
      "Ramp/idleVoltage", RampConstants.IDLE_VOLTAGE, Pair({ it.inVolts }, { it.volts })
    )

  val intakeCoralVoltageFast =
    LoggedTunableValue(
      "Ramp/intakeCoralVoltageFast",
      RampConstants.INTAKE_CORAL_VOLTAGE_FAST,
      Pair({ it.inVolts }, { it.volts })
    )

  val intakeCoralVoltageSlow =
    LoggedTunableValue(
      "Ramp/intakeCoralVoltageSlow",
      RampConstants.INTAKE_CORAL_VOLTAGE_SLOW,
      Pair({ it.inVolts }, { it.volts })
    )
}
