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
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds

class TwoNoteCenterlineFromSourceAutoPath(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand().withTimeout(0.5),
      WaitCommand(0.25),
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
                Translation2d(2.17.meters, 2.12.meters).translation2d,
                null,
                ((startingPose.rotation.inDegrees + 180) / 2).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.2.meters, 0.77.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        )
          .withTimeout(3.828 + 0.25),
        WaitCommand(2.0).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.2.meters, 0.77.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.17.meters, 2.12.meters).translation2d,
                null,
                ((startingPose.rotation.inDegrees + 180) / 2).degrees.inRotation2ds
              ),
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                startingPose.rotation.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(0.5).andThen(superstructure.prepSpeakerLowCommand())
      ),
      superstructure.scoreCommand().withTimeout(0.5),
      WaitCommand(0.25),
    )
  }

  companion object {
    val startingPose = Pose2d(0.63.meters, 4.44.meters, 120.degrees)
  }
}
