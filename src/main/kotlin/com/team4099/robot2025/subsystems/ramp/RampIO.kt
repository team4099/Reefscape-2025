package com.team4099.robot2025.subsystems.rollers

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

interface RampIO {
  class RampIOInputs : LoggableInputs {
    // Roller Inputs
    var velocity = 0.rotations.perMinute
    var appliedVoltage = 0.0.volts
    var statorCurrent = 0.0.amps
    var supplyCurrent = 0.0.amps
    var motorTemp = 0.0.celsius
    var beamBroken = false
    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("velocityRPM", velocity.inRotationsPerMinute)
      table?.put("appliedVoltage", appliedVoltage.inVolts)
      table?.put("statorCurrentAmps", statorCurrent.inAmperes)
      table?.put("supplyCurrentAmps", supplyCurrent.inAmperes)
      table?.put("tempCelsius", motorTemp.inCelsius)
      table?.put("beamBroken", beamBroken)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("velocityRPM", velocity.inRotationsPerMinute)?.let {
        velocity = it.rotations.perMinute
      }
      table?.get("appliedVoltage", appliedVoltage.inVolts)?.let { appliedVoltage = it.volts }
      table?.get("statorCurrentAmps", statorCurrent.inAmperes)?.let { statorCurrent = it.amps }
      table?.get("supplyCurrentAmps", supplyCurrent.inAmperes)?.let { supplyCurrent = it.amps }
      table?.get("tempCelsius", motorTemp.inCelsius)?.let { motorTemp = it.celsius }
      table?.get("beamroken", beamBroken)?.let { beamBroken = it }
    }
  }

  fun updateInputs(inputs: RampIOInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}
}
