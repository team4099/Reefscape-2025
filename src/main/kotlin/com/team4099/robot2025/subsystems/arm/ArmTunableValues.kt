package com.team4099.robot2025.subsystems.arm

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ArmConstants
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts

object ArmTunableValues {
  val armKP = LoggedTunableValue("Arm/armKP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))

  val armKI =
    LoggedTunableValue(
      "Arm/armKI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )

  val armKD =
    LoggedTunableValue(
      "Arm/armKD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object ArmAngles {

    val idleAngle =
      LoggedTunableValue(
        "Arm/idleAngle", ArmConstants.IDLE_ANGLE, Pair({ it.inDegrees }, { it.degrees })
      )

    val idleCoralAngle =
      LoggedTunableValue(
        "Arm/idleCoralAngle",
        ArmConstants.IDLE_CORAL_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val idleAlgaeAngle =
      LoggedTunableValue(
        "Arm/idleAlgaeAngle",
        ArmConstants.IDLE_ALGAE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val safeElevatorFrontAngle =
      LoggedTunableValue(
        "Arm/safeElevatorFrontAngle",
        ArmConstants.SAFE_ELEVATOR_FRONT_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val safeElevatorBackAngle =
      LoggedTunableValue(
        "Arm/safeElevatorBackAngle",
        ArmConstants.SAFE_ELEVATOR_BACK_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeCoralAngle =
      LoggedTunableValue(
        "Arm/intakeCoralAngle",
        ArmConstants.INTAKE_CORAL_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeAlgaeGroundAngle =
      LoggedTunableValue(
        "Arm/intakeAlgaeGroundAngle",
        ArmConstants.INTAKE_ALGAE_GROUND_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeAlgaeL2Angle =
      LoggedTunableValue(
        "Arm/intakeAlgaeL2Angle",
        ArmConstants.INTAKE_ALGAE_L2_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val intakeAlgaeL3Angle =
      LoggedTunableValue(
        "Arm/intakeAlgaeL3Angle",
        ArmConstants.INTAKE_ALGAE_L3_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreCoralL1Angle =
      LoggedTunableValue(
        "Arm/scoreCoralL1Angle",
        ArmConstants.SCORE_CORAL_L1_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreCoralL2Angle =
      LoggedTunableValue(
        "Arm/scoreCoralL2Angle",
        ArmConstants.SCORE_CORAL_L2_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreCoralL3Angle =
      LoggedTunableValue(
        "Arm/scoreCoralL3Angle",
        ArmConstants.SCORE_CORAL_L3_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreCoralL4Angle =
      LoggedTunableValue(
        "Arm/scoreCoralL4Angle",
        ArmConstants.SCORE_CORAL_L4_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreAlgaeProcessorAngle =
      LoggedTunableValue(
        "Arm/scoreAlgaeProcessorAngle",
        ArmConstants.SCORE_ALGAE_PROCESSOR_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreAlgaeBargeAngle =
      LoggedTunableValue(
        "Arm/scoreAlgaeBargeAngle",
        ArmConstants.SCORE_ALGAE_BARGE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val ejectAngle =
      LoggedTunableValue(
        "Arm/ejectAngle", ArmConstants.EJECT_ANGLE, Pair({ it.inDegrees }, { it.degrees })
      )
  }
}
