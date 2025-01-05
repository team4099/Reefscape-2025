package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command

class LockDriveCommand(val drivetrain: Drivetrain) : Command() {

  override fun execute() {
    drivetrain.defaultCommand.end(true)
    drivetrain.currentRequest = Request.DrivetrainRequest.LockWheels()
  }

  override fun isFinished(): Boolean {
    return false
  }
}
