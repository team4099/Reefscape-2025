package com.team4099.robot2025.subsystems.rollers

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.RollersConstants
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object RollersTunableValues {
  val idleVoltage = LoggedTunableValue("Rollers/idleVoltage", Pair({ it.inVolts }, { it.volts }))

  val idleCoralVoltage =
    LoggedTunableValue("Rollers/idleCoralVoltage", Pair({ it.inVolts }, { it.volts }))

  val idleAlgaeVoltage =
    LoggedTunableValue("Rollers/idleAlgaeVoltage", Pair({ it.inVolts }, { it.volts }))

  val intakeCoralVoltage =
    LoggedTunableValue("Rollers/intakeCoralVoltage", Pair({ it.inVolts }, { it.volts }))

  val intakeAlgaeVoltage =
    LoggedTunableValue("Rollers/intakeAlgaeVoltage", Pair({ it.inVolts }, { it.volts }))

  val scoreBargeAlgaeVoltage =
    LoggedTunableValue(
      "Rollers/scoreBargeAlgaeVoltage",
      RollersConstants.SCORE_BARGE_ALGAE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val scoreProcessorAlgaeVoltage =
    LoggedTunableValue(
      "Rollers/scoreBargeAlgaeVoltage",
      RollersConstants.SCORE_PROCESSOR_ALGAE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val scoreCoralVoltage =
    LoggedTunableValue(
      "Rollers/scoreCoralVoltage",
      RollersConstants.SCORE_CORAL_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val coralSpitTime =
    LoggedTunableValue(
      "Rollers/coralSpitTime",
      RollersConstants.CORAL_SPIT_TIME,
      Pair({ it.inSeconds }, { it.seconds })
    )

  val algaeProcessorSpitTime =
    LoggedTunableValue(
      "Rollers/algaeProcessorSpitTime",
      RollersConstants.ALGAE_PROCESSOR_SPIT_TIME,
      Pair({ it.inSeconds }, { it.seconds })
    )

  val algaeBargeSpitTime =
    LoggedTunableValue(
      "Rollers/algaeBargeSpitTime",
      RollersConstants.ALGAE_BARGE_SPIT_TIME,
      Pair({ it.inSeconds }, { it.seconds })
    )

  val ejectVoltage =
    LoggedTunableValue(
      "Rollers/ejectVoltage",
      RollersConstants.EJECT_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )
}
