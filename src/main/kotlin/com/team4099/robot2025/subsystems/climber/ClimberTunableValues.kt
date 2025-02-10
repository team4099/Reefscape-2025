package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ClimberConstants
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
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
  val kS =
    LoggedTunableValue("Climber/kS", ClimberConstants.PID.KS, Pair({ it.inVolts }, { it.volts }))

  val kV =
    LoggedTunableValue(
      "Climber/kV",
      ClimberConstants.PID.KV,
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  val kA =
    LoggedTunableValue(
      "Climber/kA",
      ClimberConstants.PID.KA,
      Pair({ it.inVoltsPerDegreePerSecondPerSecond }, { it.volts.perDegreePerSecondPerSecond })
    )

  val kGDefault =
    LoggedTunableValue(
      "Climber/kGDefault", ClimberConstants.PID.KG_DEFAULT, Pair({ it.inVolts }, { it.volts })
    )
  val kGUnLatched =
    LoggedTunableValue(
      "Climber/kGUnLatch",
      ClimberConstants.PID.KG_UNLATCHED,
      Pair({ it.inVolts }, { it.volts })
    )
  val kGLatched =
    LoggedTunableValue(
      "Climber/kGLatched", ClimberConstants.PID.KG_LATCHED, Pair({ it.inVolts }, { it.volts })
    )

  val kP =
    LoggedTunableValue(
      "Climber/kP",
      ClimberConstants.PID.KP_REAL,
      Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  val kI =
    LoggedTunableValue(
      "Climber/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  val kD =
    LoggedTunableValue(
      "Climber/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  val climbIdleAngle =
    LoggedTunableValue(
      "Climber/climbIdleAngle",
      ClimberConstants.CLIMBER_IDLE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  val climbExtendAngle =
    LoggedTunableValue(
      "Climber/climbExtendAngle",
      ClimberConstants.CLIMBER_EXTEND_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  val climbRetractAngle =
    LoggedTunableValue(
      "Climber/climbRetractAngle",
      ClimberConstants.CLIMBER_RETRACT_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )

  val climberTestAngle =
    LoggedTunableValue(
      "Climber/testAngle",
      ClimberConstants.CLIMBER_TEST_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )
}
