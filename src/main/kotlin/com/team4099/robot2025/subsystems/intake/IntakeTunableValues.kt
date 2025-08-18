package com.team4099.robot2025.subsystems.intake

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.IntakeConstants
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreePerSecondPerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts

object IntakeTunableValues {
  val kS =
    LoggedTunableValue("Intake/kS", IntakeConstants.PID.INTAKE_KS, Pair({ it.inVolts }, { it.volts }))
  val kV =
    LoggedTunableValue(
      "Intake/kV",
      IntakeConstants.PID.INTAKE_KV,
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )
  val kA =
    LoggedTunableValue(
      "Intake/kA",
      IntakeConstants.PID.INTAKE_KA,
      Pair({ it.inVoltsPerDegreePerSecondPerSecond }, { it.volts.perDegreePerSecondPerSecond })
    )

  val kGDefault =
    LoggedTunableValue(
      "Intake/kGDefault", IntakeConstants.PID.INTAKE_KG, Pair({ it.inVolts }, { it.volts })
    )

  val kP =
    LoggedTunableValue(
      "Intake/kP",
      IntakeConstants.PID.REAL_INTAKE_KP,
      Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  val kI =
    LoggedTunableValue(
      "Intake/kI",
      IntakeConstants.PID.REAL_INTAKE_KI,
      Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  val kD =
    LoggedTunableValue(
      "Intake/kD",
      IntakeConstants.PID.REAL_INTAKE_KD,
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )
}