package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.vision.Vision
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.*

class AutoAim(val drivetrain: Drivetrain, val vision: Vision) {
  val elevatorHeightOffsetInterpolationMetersTable: InterpolatingDoubleTreeMap =
    InterpolatingDoubleTreeMap()

  val tunableElevatorOffsetInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Meter>>>

  val interpolationTestDistance =
    LoggedTunableValue("com.team4099.robot2025.commands.drivetrain.AutoAim/TestDistance", 0.0.meters, Pair({ it.inInches }, { it.inches }))

  init {
    tunableElevatorOffsetInterpolationTable =
      Constants.distanceElevatorHeightOffsetTableReal.mapIndexed { i, it ->
        Pair(
          LoggedTunableValue(
            "com.team4099.robot2025.commands.drivetrain.AutoAim/ElevatorInterpolation/$i/Distance",
            it.first,
            Pair({ it.inInches }, { it.inches })
          ),
          LoggedTunableValue(
            "com.team4099.robot2025.commands.drivetrain.AutoAim/ElevatorInterpolation/$i/Height",
            it.second,
            Pair({ it.inInches }, { it.inches })
          )
        )
      }

    updateElevatorInterpolationTable()
  }

  fun calculateElevatorHeight(): Length {
    return elevatorHeightOffsetInterpolationMetersTable.get(calculateDistanceFromSpeaker().inMeters).meters
  }

  fun periodic() {
    for (point in tunableElevatorOffsetInterpolationTable) {
      if (point.first.hasChanged() || point.second.hasChanged()) {
        updateElevatorInterpolationTable()
        break
      }
    }

    Logger.recordOutput(
      "com.team4099.robot2025.commands.drivetrain.AutoAim/InterpolatedElevatorHeight",
      elevatorHeightOffsetInterpolationMetersTable.get(interpolationTestDistance.get().inMeters)
    )
  }


  fun updateElevatorInterpolationTable() {
    elevatorHeightOffsetInterpolationMetersTable.clear()
    tunableElevatorOffsetInterpolationTable.forEach {
      elevatorHeightOffsetInterpolationMetersTable.put(it.first.get().inMeters, it.second.get().inMeters)
    }
  }

  fun calculateDistanceFromSpeaker(): Length {
    val distance = vision.distanceToTarget
    Logger.recordOutput("AutoAim/currentDistanceInches", distance.inInches)
    return distance
  }
}
