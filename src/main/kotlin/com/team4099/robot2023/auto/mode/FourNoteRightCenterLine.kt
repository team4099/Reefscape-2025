package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteRightCenterLine(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
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
                Translation2d(2.25.meters, 6.99.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.76.meters, 6.79.meters).translation2d,
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
                Translation2d(0.76.meters, 6.79.meters).translation2d,
                null,
                -120.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(7.0.meters, 7.4.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.76.meters, 6.79.meters).translation2d,
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
                Translation2d(0.76.meters, 6.79.meters).translation2d,
                null,
                -120.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(5.71.meters, 6.41.meters).translation2d,
                0.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(7.5.meters, 5.81.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(5.71.meters, 6.41.meters).translation2d,
                180.degrees.inRotation2ds,
                180.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(0.76.meters, 6.79.meters).translation2d,
                null,
                -120.degrees.inRotation2ds
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
    val startingPose = Pose2d(Translation2d(0.76.meters, 6.79.meters), -120.degrees)
  }
}
