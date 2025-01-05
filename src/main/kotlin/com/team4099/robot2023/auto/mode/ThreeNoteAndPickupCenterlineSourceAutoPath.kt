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
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

class ThreeNoteAndPickupCenterlineSourceAutoPath(
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
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.32.meters, 0.78.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.groundIntakeCommand())
      ),
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        {
          listOf(
            FieldWaypoint(
              Translation2d(8.32.meters, 0.78.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(4.33.meters, 1.67.meters).translation2d,
              null,
              (180 - 43.37583640633171).degrees.inRotation2ds
            ),
          )
        }
      ),
      superstructure.prepManualSpeakerCommand(-4.5.degrees, 4000.rotations.perMinute),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  (4.33.meters + 8.32.meters) / 2,
                  (2.44 + 0.78).meters / 2 - 0.1.meters
                )
                  .translation2d,
                null,
                ((180 - 43.3758 + 210) / 2 + 10).degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(8.32.meters, 2.44.meters).translation2d,
                null,
                210.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  (4.33.meters + 8.32.meters) / 2,
                  (2.44 + 0.78).meters / 2 + 0.1.meters
                )
                  .translation2d,
                null,
                ((180 - 43.3758 + 210) / 2 + 10).degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-4.5.degrees, 4000.rotations.perMinute),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.84.meters, 4.09.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.32.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.groundIntakeCommand())
      )
    )
  }

  companion object {
    val startingPose = Pose2d(0.63.meters, 4.44.meters, 120.degrees)
  }
}
