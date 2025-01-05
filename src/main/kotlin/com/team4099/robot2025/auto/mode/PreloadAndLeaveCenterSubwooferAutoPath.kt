package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class PreloadAndLeaveCenterSubwooferAutoPath(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  secondaryWaitTime: Time
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
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
              Translation2d(2.87.meters, 6.27.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(8.16.meters, 6.69.meters).translation2d,
              null,
              180.degrees.inRotation2ds
            )
          )
        }
      )
    )
  }

  companion object {
    val startingPose = Pose2d(1.41.meters, 5.55.meters, 180.degrees)
  }
}
