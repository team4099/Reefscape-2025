package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FiveNoteCenterlineAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      ResetPoseCommand(drivetrain, Pose2d(Translation2d(1.51.meters, 5.49.meters), 180.degrees)),
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        {
          listOf(
            FieldWaypoint(
              Translation2d(1.51.meters, 5.49.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Subwoofer
            FieldWaypoint(
              Translation2d(2.39.meters, 5.49.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Middle wing note
            FieldWaypoint(
              Translation2d(2.51.meters, 6.98.meters).translation2d,
              null,
              235.degrees.inRotation2ds
            ), // Leftmost wing note
            FieldWaypoint(
              Translation2d(3.meters, 6.98.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Right in front of the leftmost wing note
            FieldWaypoint(
              Translation2d(7.79.meters, 7.42.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Leftmost center line note
            FieldWaypoint(
              Translation2d(2.39.meters, 5.49.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Subwoofer
            FieldWaypoint(
              Translation2d(5.82.meters, 6.5.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // In order to avoid stage
            FieldWaypoint(
              Translation2d(7.79.meters, 5.78.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Second leftmost wing note
            FieldWaypoint(
              Translation2d(5.82.meters, 6.5.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // In order to avoid stage
            FieldWaypoint(
              Translation2d(2.39.meters, 5.49.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ), // Subwoofer
          )
        },
        resetPose = true
      )
    )
  }
}
