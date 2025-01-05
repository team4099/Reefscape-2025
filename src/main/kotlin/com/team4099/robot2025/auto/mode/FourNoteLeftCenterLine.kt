package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteLeftCenterLine(val drivetrain: Drivetrain, val superstructure: Superstructure) :
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
                Translation2d(2.39.meters, 4.15.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.76.meters, 6.79.meters).translation2d,
                null,
                60.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(0.78.meters, 4.33.meters).translation2d,
                null,
                -120.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(7.72.meters, 0.84.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.78.meters, 4.33.meters).translation2d,
                null,
                -120.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(0.78.meters, 4.33.meters).translation2d,
                null,
                60.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.88.meters, 2.0.meters).translation2d,
                0.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(7.7.meters, 2.38.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(4.88.meters, 2.0.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.78.meters, 4.33.meters).translation2d,
                null,
                60.degrees.inRotation2ds
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
    val startingPose = Pose2d(Translation2d(0.78.meters, 4.33.meters), 60.degrees)
  }
}
