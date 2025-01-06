package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.inRotation2ds

class TestDriveCommand(val drivetrain: Drivetrain) : Command() {

  override fun execute() {
    drivetrain.currentRequest =
      Request.DrivetrainRequest.ClosedLoop(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          4.0, 0.0, 0.0, drivetrain.odomTRobot.rotation.inRotation2ds
        )
      )
    CustomLogger.recordDebugOutput("ActiveCommands/TestDriveCommand", true)
  }

  override fun isFinished(): Boolean {
    return false
  }
}
