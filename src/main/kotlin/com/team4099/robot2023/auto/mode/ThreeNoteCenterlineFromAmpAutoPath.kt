package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

class ThreeNoteCenterlineFromAmpAutoPath(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand().withTimeout(0.75),
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
                Translation2d(1.90.meters, 6.76.meters).translation2d,
                null,
                210.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.87.meters, 6.27.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.27.meters + 2.feet, 7.45.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(2.25).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.27.meters + 2.feet, 7.45.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(3.9.meters, 6.45.meters).translation2d,
                null,
                (180 + 13.856).degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0)
          .andThen(
            superstructure
              .prepManualSpeakerCommand(-7.degrees, 4000.rotations.perMinute)
              .withTimeout(1.0)
          )
      ),
      superstructure.scoreCommand().withTimeout(0.5),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(3.9.meters, 6.45.meters).translation2d,
                null,
                (180 + 13.856).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((3.9 + 8.27).meters / 2, (6.45 + 7.45).meters / 2)
                  .translation2d,
                null,
                ((180 + 13.856 + 165).degrees / 2).inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.5.meters, 5.6.meters).translation2d,
                null,
                150.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.5.meters, 5.6.meters).translation2d,
                null,
                150.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((3.9 + 8.27).meters / 2, (6.45 + 7.45).meters / 2)
                  .translation2d,
                null,
                ((180 + 13.856 + 160).degrees / 2).inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(3.9.meters, 6.45.meters).translation2d,
                null,
                (180 + 13.856).degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0)
          .andThen(
            superstructure.prepManualSpeakerCommand(-7.degrees, 4000.rotations.perMinute)
          )
      ),
      superstructure.scoreCommand().withTimeout(0.5),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(3.9.meters, 6.45.meters).translation2d,
                null,
                (180 + 13.856).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((3.9 + 8.27).meters / 2, (6.45 + 7.45).meters / 2)
                  .translation2d,
                null,
                ((180 + 13.856 + 165).degrees / 2).inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.27.meters, 4.11.meters).translation2d,
                null,
                140.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.27.meters, 4.11.meters).translation2d,
                null,
                140.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((3.9 + 8.27).meters / 2, (6.45 + 7.45).meters / 2)
                  .translation2d,
                null,
                ((180 + 13.856 + 160).degrees / 2).inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(3.9.meters, 6.45.meters).translation2d,
                null,
                (180 + 13.856).degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0)
          .andThen(
            superstructure.prepManualSpeakerCommand(
              -5.5.degrees, 4000.rotations.perMinute
            )
          )
      ),
      superstructure.scoreCommand()
    )
  }

  companion object {
    val startingPose = Pose2d(0.75.meters, 6.70.meters, 240.degrees)
  }
}
