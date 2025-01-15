package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.RollersConstants
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object RollersTunableValues {
  val idleVoltage = LoggedTunableValue("Rollers/idleVoltage", Pair({ it.inVolts }, { it.volts }))

  val idleCoralVoltage =
    LoggedTunableValue("Rollers/idleCoralVoltage", Pair({ it.inVolts }, { it.volts }))

  val idleAlgaeVoltage =
    LoggedTunableValue("Rollers/idleAlgaeVoltage", Pair({ it.inVolts }, { it.volts }))

  val intakeVoltage =
    LoggedTunableValue(
      "Rollers/intakeVoltage",
      RollersConstants.INTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val scoreVoltage =
    LoggedTunableValue(
      "Rollers/scoreVoltage",
      RollersConstants.SCORE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )
}
