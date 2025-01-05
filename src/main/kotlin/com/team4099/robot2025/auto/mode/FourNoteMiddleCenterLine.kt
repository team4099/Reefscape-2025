package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteMiddleCenterLine(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
      ResetPoseCommand(drivetrain, Pose2d(startingPose.x, startingPose.y, startingPose.rotation)),
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
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
                Translation2d(2.0.meters, 5.525.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(2.4.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(2.0.meters, 5.475.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
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
                Translation2d(4.87.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(7.72.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(4.87.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
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
                Translation2d(4.87.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(7.72.meters, 2.47.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(4.87.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.scoreCommand()
    )
  }
  companion object {
    val startingPose = Pose2d(Translation2d(1.46.meters, 5.5.meters), 180.degrees)
  }
}
