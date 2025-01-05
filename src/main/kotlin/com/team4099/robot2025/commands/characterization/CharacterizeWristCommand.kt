package com.team4099.robot2023.commands

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class CharacterizeWristCommand(val wrist: Wrist, val positive: Boolean) : Command() {
  private var currentVoltage = 0.volts
  private var lastCallTime = Clock.fpgaTime

  init {
    addRequirements(wrist)
  }

  override fun execute() {
    if ((Clock.fpgaTime - lastCallTime).inMilliseconds > 500) {
      currentVoltage += if (positive) 0.01.volts else (-0.01).volts
      wrist.setWristVoltage(currentVoltage)
      lastCallTime = Clock.fpgaTime
    }
  }

  override fun isFinished(): Boolean {
    return wrist.inputs.wristVelocity.absoluteValue > 0.5.degrees.perSecond
  }

  override fun end(interrupted: Boolean) {
    wrist.setWristVoltage(0.0.volts)
  }
}
