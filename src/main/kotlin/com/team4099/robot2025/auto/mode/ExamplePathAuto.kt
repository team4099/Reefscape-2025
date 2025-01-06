package com.team4099.robot2025.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2025.commands.drivetrain.DrivePathCommand
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        {
          listOf(
            FieldWaypoint(
              startingPose.translation.translation2d,
              null,
              startingPose.rotation.inRotation2ds
            ),
            FieldWaypoint(
              startingPose.translation.translation2d,
              null,
              startingPose.rotation.inRotation2ds
            )
          )
        }
      )
    )
  }
  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters, 5.535.meters), 180.degrees)
  }
}
