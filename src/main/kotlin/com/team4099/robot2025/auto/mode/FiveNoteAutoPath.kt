package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

class FiveNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        WaitCommand(0.1).andThen(superstructure.groundIntakeCommand()),
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.4.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.8.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        )
      ),
      superstructure.prepManualSpeakerCommand(
        -4.5.degrees, 3000.rotations.perMinute, 0.6.degrees
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.8.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((2.84.meters + 8.3.meters) / 2, 6.45.meters).translation2d,
                null,
                170.degrees.inRotation2ds
              ), // In order to avoid stage
              FieldWaypoint(
                Translation2d(8.3.meters, 5.75.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ), // Second leftmost centerline note
              FieldWaypoint(
                Translation2d((2.84.meters + 8.3.meters) / 2, 6.55.meters).translation2d,
                null,
                170.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        superstructure
          .scoreCommand()
          .withTimeout(0.5)
          .andThen(WaitCommand(0.75))
          .andThen(superstructure.groundIntakeCommand())
          .andThen(WaitCommand(0.25))
          .andThen(
            superstructure.prepManualSpeakerCommand(
              -4.5.degrees, 3000.rotations.perMinute, 0.6.degrees
            )
          )
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.3.meters, 6.4.meters).translation2d,
                null,
                200.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.8.meters, (6.85 + 0.1).meters).translation2d,
                null,
                207.89.degrees.inRotation2ds
              ),
            )
          }
        ),
        superstructure
          .groundIntakeCommand()
          .andThen(
            superstructure.prepManualSpeakerCommand(
              -3.degrees, 3000.rotations.perMinute, 0.7.degrees
            )
          )
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.8.meters, (6.85 + 0.1).meters).translation2d,
                null,
                207.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.25.meters, 4.9.meters).translation2d,
                null,
                158.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.75.meters, 4.15.meters).translation2d,
                null,
                152.degrees.inRotation2ds
              ),
            )
          }
        ),
        superstructure
          .scoreCommand()
          .andThen(WaitCommand(0.1))
          .andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-3.degrees, 3000.rotations.perMinute),
      superstructure.scoreCommand().withTimeout(0.5)
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.4.meters, 5.50.meters), 180.degrees)
  }
}
