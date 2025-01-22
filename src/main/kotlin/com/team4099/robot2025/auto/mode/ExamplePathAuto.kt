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

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        TrajectoryTypes.Choreo(
          Choreo.loadTrajectory<SwerveSample>("exampleTraj").get()
        )
      )
    )
  }
  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters, 5.535.meters), 180.degrees)
  }
}
