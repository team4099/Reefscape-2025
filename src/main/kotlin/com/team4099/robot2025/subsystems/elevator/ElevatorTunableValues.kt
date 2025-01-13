package com.team4099.robot2025.subsystems.elevator

import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ElevatorConstants
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.perInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorTunableValues {

  val slot0kP = LoggedTunableValue("Elevator/slot0kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  val slot0kI =
    LoggedTunableValue(
      "Elevator/slot0kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  val slot0kD =
    LoggedTunableValue(
      "Elevator/slot0kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  val slot1kP =
    LoggedTunableValue("Elevator/slot1kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  val slot1kI =
    LoggedTunableValue(
      "Elevator/slot1kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  val slot1kD =
    LoggedTunableValue(
      "Elevator/slot1kD",
      Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  val slot2kP =
    LoggedTunableValue("Elevator/slot2kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
  val slot2kI =
    LoggedTunableValue(
      "Elevator/slot2kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
  val slot2kD =
    LoggedTunableValue(
      "Elevator/slot2kD",
      Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  object TunableElevatorHeights {
    val enableElevator =
      LoggedTunableNumber(
        "Elevator/enableMovementElevator", if (ElevatorConstants.ENABLE_ELEVATOR) 1.0 else 0.0
      )

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

    val yPos =
      LoggedTunableValue(
        "Elevator/yPos",
        0.0.meters,
      )

    val y1Pos =
      LoggedTunableValue(
        "Elevator/y1Pos",
        0.0.meters,
      )
  }
}
