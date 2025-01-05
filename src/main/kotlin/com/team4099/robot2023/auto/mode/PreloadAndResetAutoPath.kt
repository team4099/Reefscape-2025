package com.team4099.robot2023.auto.mode

import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees

class PreloadAndResetAutoPath(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  secondaryWaitTime: Time
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
      superstructure.scoreCommand(),
      WaitCommand(secondaryWaitTime.inSeconds),
      ResetPoseCommand(drivetrain, startingPose)
    )
  }

  companion object {
    val startingPose = Pose2d(1.41.meters, 5.55.meters, 180.degrees)
  }
}
