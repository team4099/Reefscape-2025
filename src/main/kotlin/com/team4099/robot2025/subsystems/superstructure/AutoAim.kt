package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.SuperstructureConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.vision.Vision
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import kotlin.math.absoluteValue

class AutoAim(val drivetrain: Drivetrain, val vision: Vision) {
  val flywheelSpeedRPMInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()
  val wristAngleDegreesInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

  val highFlywheelSpeedRPMInterpolationTable: InterpolatingDoubleTreeMap =
    InterpolatingDoubleTreeMap()
  val highWristAngleDegreesInterpolationTable: InterpolatingDoubleTreeMap =
    InterpolatingDoubleTreeMap()

  val tunableFlywheelInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Velocity<Radian>>>>

  val tunableWristInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Radian>>>

  val tunableHighFlywheelInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Velocity<Radian>>>>

  val tunableHighWristInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Radian>>>

  val interpolationTestDistance =
    LoggedTunableValue("AutoAim/TestDistance", 0.0.meters, Pair({ it.inInches }, { it.inches }))

  init {

    if (RobotBase.isReal()) {
      tunableFlywheelInterpolationTable =
        SuperstructureConstants.distanceFlywheelSpeedTableReal.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/SpeedRPM",
              it.second,
              Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
            )
          )
        }

      tunableWristInterpolationTable =
        SuperstructureConstants.distanceWristAngleTableReal.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/AngleDegrees",
              it.second,
              Pair({ it.inDegrees }, { it.degrees })
            )
          )
        }
    } else {
      tunableFlywheelInterpolationTable =
        SuperstructureConstants.distanceFlywheelSpeedTableSim.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/SpeedRPM",
              it.second,
              Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
            )
          )
        }

      tunableWristInterpolationTable =
        SuperstructureConstants.distanceWristAngleTableSim.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/AngleDegrees",
              it.second,
              Pair({ it.inDegrees }, { it.degrees })
            )
          )
        }
    }

    tunableHighFlywheelInterpolationTable =
      SuperstructureConstants.highDistanceFlywheelSpeedTableReal.mapIndexed { i, it ->
        Pair(
          LoggedTunableValue(
            "AutoAim/HighFlywheelInterpolation/$i/Distance",
            it.first,
            Pair({ it.inInches }, { it.inches })
          ),
          LoggedTunableValue(
            "AutoAim/HighFlywheelInterpolation/$i/SpeedRPM",
            it.second,
            Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
          )
        )
      }

    tunableHighWristInterpolationTable =
      SuperstructureConstants.highDistanceWristAngleTableReal.mapIndexed { i, it ->
        Pair(
          LoggedTunableValue(
            "AutoAim/HighWristInterpolation/$i/Distance",
            it.first,
            Pair({ it.inInches }, { it.inches })
          ),
          LoggedTunableValue(
            "AutoAim/HighWristInterpolation/$i/AngleDegrees",
            it.second,
            Pair({ it.inDegrees }, { it.degrees })
          )
        )
      }

    updateFlywheelInterpolationTable()
    updateWristInterpolationTable()

    updateHighFlywheelInterpolationTable()
    updateHighWristInterpolationTable()
  }

  fun periodic() {
    for (point in tunableFlywheelInterpolationTable) {
      if (point.first.hasChanged() || point.second.hasChanged()) {
        updateFlywheelInterpolationTable()
        break
      }
    }

    for (point in tunableWristInterpolationTable) {
      if (point.first.hasChanged() || point.second.hasChanged()) {
        updateWristInterpolationTable()
        break
      }
    }

    Logger.recordOutput(
      "AutoAim/InterpolatedFlywheelSpeed",
      flywheelSpeedRPMInterpolationTable.get(interpolationTestDistance.get().inMeters)
    )

    Logger.recordOutput(
      "AutoAim/InterpolatedWristAngle",
      wristAngleDegreesInterpolationTable.get(interpolationTestDistance.get().inMeters)
    )
  }

  fun calculateFlywheelSpeed(): AngularVelocity {
    return flywheelSpeedRPMInterpolationTable.get(calculateDistanceFromSpeaker().inMeters)
      .rotations
      .perMinute
  }

  fun calculateWristAngle(): Angle {
    return wristAngleDegreesInterpolationTable.get(calculateDistanceFromSpeaker().inMeters).degrees
  }

  fun calculateHighFlywheelSpeed(): AngularVelocity {
    return highFlywheelSpeedRPMInterpolationTable.get(calculateDistanceFromSpeaker().inMeters)
      .rotations
      .perMinute
  }

  fun calculateHighWristAngle(): Angle {
    return highWristAngleDegreesInterpolationTable.get(calculateDistanceFromSpeaker().inMeters)
      .degrees
  }

  fun updateFlywheelInterpolationTable() {
    flywheelSpeedRPMInterpolationTable.clear()
    tunableFlywheelInterpolationTable.forEach {
      flywheelSpeedRPMInterpolationTable.put(
        it.first.get().inMeters, it.second.get().inRotationsPerMinute
      )
    }
  }

  fun updateHighFlywheelInterpolationTable() {
    highFlywheelSpeedRPMInterpolationTable.clear()
    tunableHighFlywheelInterpolationTable.forEach {
      highFlywheelSpeedRPMInterpolationTable.put(
        it.first.get().inMeters, it.second.get().inRotationsPerMinute
      )
    }
  }

  fun updateWristInterpolationTable() {
    wristAngleDegreesInterpolationTable.clear()
    tunableWristInterpolationTable.forEach {
      wristAngleDegreesInterpolationTable.put(it.first.get().inMeters, it.second.get().inDegrees)
    }
  }

  fun updateHighWristInterpolationTable() {
    highWristAngleDegreesInterpolationTable.clear()
    tunableHighWristInterpolationTable.forEach {
      highWristAngleDegreesInterpolationTable.put(
        it.first.get().inMeters, it.second.get().inDegrees
      )
    }
  }

  fun calculateDistanceFromSpeaker(): Length {
    val distance =
      Translation2d(
        vision.robotTSpeaker.y -
          (drivetrain.robotVelocity.y * vision.robotTSpeaker.norm.absoluteValue / 7)
            .value
            .meters,
        vision.robotTSpeaker.x -
          (drivetrain.robotVelocity.x * vision.robotTSpeaker.norm.absoluteValue / 7)
            .value
            .meters
      )
        .magnitude
        .meters
    Logger.recordOutput("AutoAim/currentDistanceInches", distance.inInches)
    return distance
  }
}
