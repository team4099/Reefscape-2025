package com.team4099.robot2023.commands

import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ShooterCharacterizationCommand(val wrist: Wrist) : Command() {
  var voltage = 0.0.volts
  init {
    addRequirements(wrist)
  }

  override fun execute() {
    voltage += 0.01.volts
    wrist.currentRequest = Request.WristRequest.OpenLoop(voltage)
  }

  override fun isFinished(): Boolean {
    return wrist.inputs.wristVelocity > 0.05.degrees.perSecond
  }

  override fun end(interrupted: Boolean) {
    wrist.currentRequest = Request.WristRequest.OpenLoop(0.volts)
  }
}
