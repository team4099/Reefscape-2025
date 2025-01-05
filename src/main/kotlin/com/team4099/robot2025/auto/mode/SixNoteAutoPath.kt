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
import org.team4099.lib.units.perSecond

class SixNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.37.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.65.meters, 4.26.meters).translation2d,
                null,
                152.degrees.inRotation2ds
              ),
            )
          },
        ),
        WaitCommand(0.25).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-2.degrees),
      superstructure.scoreCommand().withTimeout(0.5),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.65.meters, 4.26.meters).translation2d,
                null,
                152.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.2.meters, 5.0.meters).translation2d,
                null,
                210.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
        ),
        WaitCommand(0.25).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-3.degrees),
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
                Translation2d(2.2.meters, 6.4.meters).translation2d,
                null,
                200.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.75.meters, 6.85.meters).translation2d,
                null,
                207.89.degrees.inRotation2ds
              ),
            )
          },
        ),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-2.degrees),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.75.meters, 6.85.meters).translation2d,
                null,
                207.89.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.3.meters, 5.75.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(5.8.meters, 6.3.meters).translation2d,
                null,
                185.degrees.inRotation2ds
              ),
            )
          },
        ),
        WaitCommand(0.5)
          .andThen(superstructure.groundIntakeCommand())
          .andThen(WaitCommand(1.5))
          .andThen(
            superstructure.prepManualSpeakerCommand(7.degrees, 4500.rotations.perSecond)
          ),
      ),
      superstructure.scoreCommand().withTimeout(0.5),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(5.8.meters, 6.3.meters).translation2d,
                null,
                185.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(6.2.meters, 6.2.meters).translation2d,
                null,
                175.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.0.meters, 4.0.meters).translation2d,
                null,
                155.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(6.15.meters, 6.2.meters).translation2d,
                null,
                175.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(5.8.meters, 6.3.meters).translation2d,
                null,
                185.degrees.inRotation2ds
              )
            )
          },
        ),
        WaitCommand(0.5)
          .andThen(superstructure.groundIntakeCommand())
          .andThen(WaitCommand(0.5))
          .andThen(
            superstructure.prepManualSpeakerCommand(7.degrees, 4500.rotations.perSecond)
          ),
      ),
      superstructure.scoreCommand().withTimeout(0.5)
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.37.meters, 5.50.meters), 180.degrees)
  }
}
