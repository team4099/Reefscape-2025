package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class DriveBrakeModeCommand(val drivetrain: Drivetrain) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.currentRequest =
      DrivetrainRequest.OpenLoop(
        0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
      )
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(true) }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(false) }
  }
}
