package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
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

class FourNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand().withTimeout(0.5),
      WaitCommand(0.5),
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
                Translation2d(2.91.meters - 0.75.meters, 6.82.meters).translation2d,
                null,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(2.91.meters - 0.25.meters, 7.meters).translation2d,
                0.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        )
          .withTimeout(3.235 + 0.5),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure
        .scoreCommand()
        .withTimeout(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds + 0.5),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                180.degrees.inRotation2ds
              ), // Subwoofer
              FieldWaypoint(
                Translation2d(2.41.meters - 0.4.meters, 4.4.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.41.meters, 4.1.meters).translation2d,
                0.degrees.inRotation2ds,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        )
          .withTimeout(3.235 + 0.5),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure
        .scoreCommand()
        .withTimeout(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds + 0.5),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  ((1.43.meters) + (2.34.meters + 0.3.meters)) / 2 +
                    0.25.meters,
                  5.55.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.34.meters + 0.3.meters + 0.25.meters, 5.535.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  ((1.43.meters) + (2.34.meters + 0.3.meters)) / 2 +
                    0.25.meters,
                  5.45.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                180.degrees.inRotation2ds
              ) // Subwoofer
            )
          }
        )
          .withTimeout(3.235 + 0.5),
        WaitCommand(0.3).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure
        .scoreCommand()
        .withTimeout(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds + 0.5),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d, null, 180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.35.meters, 4.85.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(5.92.meters, 3.9.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.29.meters, 4.09.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      )
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters + 1.inches, 5.535.meters), 180.degrees)
  }
}
