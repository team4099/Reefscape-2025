package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class OpenLoopReverseCommand(val drivetrain: Drivetrain) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.currentRequest =
      DrivetrainRequest.OpenLoop(
        0.degrees.perSecond,
        Pair(-5.0.feet.perSecond, 0.0.feet.perSecond),
        fieldOriented = false
      )
  }

  override fun isFinished(): Boolean {
    return false
  }
}
