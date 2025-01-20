package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.logging.LoggedTunableValue
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

object ClimberTunableValues {
  val kS = LoggedTunableValue("Climber/kS", Pair({ it.inVolts }, { it.volts }))

  val kV =
    LoggedTunableValue(
      "Climber/kV", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  val kA =
    LoggedTunableValue(
      "Climber/kA",
      Pair({ it.inVoltsPerDegreePerSecondPerSecond }, { it.volts.perDegreePerSecondPerSecond })
    )

  val kG = LoggedTunableValue("Climber/kG", Pair({ it.inVolts }, { it.volts }))

  val kPSlot0 =
    LoggedTunableValue("Climber/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))

  val kISlot0 =
    LoggedTunableValue(
      "Climber/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

  val kDSlot0 =
    LoggedTunableValue(
      "Climber/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  val kPSlot1 =
    LoggedTunableValue("Climber/kPSlot1", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))

  val kISlot1 =
    LoggedTunableValue(
      "Climber/kISlot1", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

  val kDSlot1 =
    LoggedTunableValue(
      "Climber/kDSlot1",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )
}
