package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class DriveModuleSteeringCommand(val drivetrain: Drivetrain) : Command() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors()
  }

  override fun isFinished(): Boolean {
    return true
  }

  override fun execute() {
    CustomLogger.recordDebugOutput("ActiveCommands/ZeroSensorsCommand", true)
  }
}
