package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ElevatorConstants
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.perMeterPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorTunableValues {
  val enableElevator =
    LoggedTunableNumber(
      "Elevator/enableMovementElevator", if (ElevatorConstants.ENABLE_ELEVATOR) 1.0 else 0.0
    )

  val kP = LoggedTunableValue("Elevator/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  val kI =
    LoggedTunableValue(
      "Elevator/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  val kD =
    LoggedTunableValue(
      "Elevator/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )
  val kGFirst = LoggedTunableValue("Elevator/kG", Pair({ it.inVolts }, { it.volts }))
  val kSFirst = LoggedTunableValue("Elevator/kS", Pair({ it.inVolts }, { it.volts }))
  val kVFirst =
    LoggedTunableValue(
      "Elevator/kV", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )
  val kAFirst =
    LoggedTunableValue(
      "Elevator/kA",
      Pair({ it.inVoltsPerMeterPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )
  val kGSecond = LoggedTunableValue("Elevator/kG", Pair({ it.inVolts }, { it.volts }))
  val kSSecond = LoggedTunableValue("Elevator/kS", Pair({ it.inVolts }, { it.volts }))
  val kVSecond =
    LoggedTunableValue(
      "Elevator/kV", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )
  val kASecond =
    LoggedTunableValue(
      "Elevator/kA",
      Pair({ it.inVoltsPerMeterPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )
  val kGThird = LoggedTunableValue("Elevator/kG", Pair({ it.inVolts }, { it.volts }))
  val kSThird = LoggedTunableValue("Elevator/kS", Pair({ it.inVolts }, { it.volts }))
  val kVThird =
    LoggedTunableValue(
      "Elevator/kV", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )
  val kAThird =
    LoggedTunableValue(
      "Elevator/kA",
      Pair({ it.inVoltsPerMeterPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  object TunableElevatorHeights {
    val minPosition =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val maxPosition =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.UPWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val testPosition =
      LoggedTunableValue(
        "Elevator/testPosition",
        ElevatorConstants.L2_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val openLoopExtendVoltage =
      LoggedTunableValue(
        "Elevator/openLoopExtendVoltage",
        ElevatorConstants.OPEN_LOOP_EXTEND_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val openLoopRetractVoltage =
      LoggedTunableValue(
        "Elevator/openLoopRetractVoltage",
        ElevatorConstants.OPEN_LOOP_RETRACT_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )

    val L1Height =
      LoggedTunableValue(
        "Elevator/L1Height", ElevatorConstants.L1_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L2Height =
      LoggedTunableValue(
        "Elevator/L2Height", ElevatorConstants.L2_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L3Height =
      LoggedTunableValue(
        "Elevator/L3Height", ElevatorConstants.L3_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val L4Height =
      LoggedTunableValue(
        "Elevator/L4Height", ElevatorConstants.L4_HEIGHT, Pair({ it.inInches }, { it.inches })
      )

    val algaeLowHeight =
      LoggedTunableValue(
        "Elevator/algaeLowHeight",
        ElevatorConstants.ALGAE_LOW_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val algaeHighHeight =
      LoggedTunableValue(
        "Elevator/algaeHighHeight",
        ElevatorConstants.ALGAE_LOW_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )
  }
}
