package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.TrajectoryTypes
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        TrajectoryTypes.Choreo(
          Choreo.loadTrajectory<SwerveSample>("examplePath").get()
        )
      )
    )
  }
  companion object {
    val startingPose = Pose2d(Translation2d(7.261556148529053.meters, 4.551422595977783.meters), 3.1145838845027867.radians)
  }
}
