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

  val kS = LoggedTunableValue("Elevator/kS", Pair({ it.inVolts }, { it.volts }))

  val kV =
    LoggedTunableValue(
      "Elevator/kV", Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

  val kA =
    LoggedTunableValue(
      "Elevator/kA",
      Pair({ it.inVoltsPerMeterPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  val kGDefault = LoggedTunableValue("Elevator/kGDefault", Pair({ it.inVolts }, { it.volts }))
  val kGFirst = LoggedTunableValue("Elevator/kGFirstStage", Pair({ it.inVolts }, { it.volts }))
  val kGSecond = LoggedTunableValue("Elevator/kGSecondStage", Pair({ it.inVolts }, { it.volts }))
  val kGThird = LoggedTunableValue("Elevator/kGThirdStage", Pair({ it.inVolts }, { it.volts }))

  val testingVoltage =
    LoggedTunableValue(
      "Elevator/testingVoltage",
      ElevatorConstants.TESTING_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  object ElevatorHeights {
    val minHeight =
      LoggedTunableValue(
        "Elevator/minPosition",
        ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val maxHeight =
      LoggedTunableValue(
        "Elevator/maxPosition",
        ElevatorConstants.UPWARDS_EXTENSION_LIMIT,
        Pair({ it.inInches }, { it.inches })
      )

    val idleHeight =
      LoggedTunableValue(
        "Elevator/idleHeight",
        ElevatorConstants.IDLE_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val idleCoralVerticalHeight =
      LoggedTunableValue(
        "Elevator/idleCoralVerticalHeight",
        ElevatorConstants.IDLE_CORAL_VERTICAL_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val idleCoralHorizontalHeight =
      LoggedTunableValue(
        "Elevator/idleCoralHorizontalHeight",
        ElevatorConstants.IDLE_CORAL_HORIZONTAL_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )


    val idleAlgaeHeight =
      LoggedTunableValue(
        "Elevator/idleAlgaeHeight",
        ElevatorConstants.IDLE_ALGAE_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val passThroughHeight =
      LoggedTunableValue(
        "Elevator/passThroughHeight",
        ElevatorConstants.PASS_THROUGH_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val intakeCoralHeight =
      LoggedTunableValue(
        "Elevator/intakeCoralHeight",
        ElevatorConstants.INTAKE_CORAL_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val intakeL1Height =
      LoggedTunableValue(
        "Elevator/intakeL1Height",
        ElevatorConstants.INTAKE_L1_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val intakeAlgaeGroundHeight =
      LoggedTunableValue(
        "Elevator/intakeAlgaeGroundHeight",
        ElevatorConstants.INTAKE_ALGAE_GROUND_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val intakeAlgaeL2Height =
      LoggedTunableValue(
        "Elevator/intakeAlgaeL2Height",
        ElevatorConstants.INTAKE_ALGAE_L2_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val intakeAlgaeL3Height =
      LoggedTunableValue(
        "Elevator/intakeAlgaeL3Height",
        ElevatorConstants.INTAKE_ALGAE_L3_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val scoreAlgaeProcessorHeight =
      LoggedTunableValue(
        "Elevator/scoreAlgaeProcessorHeight",
        ElevatorConstants.SCORE_PROCESSOR_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val scoreAlgaeBargeHeight =
      LoggedTunableValue(
        "Elevator/scoreBargeProcessorHeight",
        ElevatorConstants.SCORE_BARGE_HEIGHT,
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

    val L1HorizontalHeight =
      LoggedTunableValue(
        "Elevator/coralL1HorizontalHeight",
        ElevatorConstants.L1_HORIZONTAL_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )

    val L1VerticalHeight =
      LoggedTunableValue(
        "Elevator/coralL1VerticalHeight",
        ElevatorConstants.L1_VERTICAL_HEIGHT,
        Pair({ it.inInches }, { it.inches })
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

    val ejectHeight =
      LoggedTunableValue(
        "Elevator/ejectHeight",
        ElevatorConstants.EJECT_HEIGHT,
        Pair({ it.inInches }, { it.inches })
      )
  }
}
