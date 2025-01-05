package com.team4099.robot2023.subsystems.feeder

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface FeederIO {
  class FeederIOInputs : LoggableInputs {
    var feederVelocity = 0.0.rotations.perMinute
    var feederAppliedVoltage = 0.0.volts
    var feederStatorCurrent = 0.0.amps
    var feederSupplyCurrent = 0.0.amps
    var feederTemp = 0.0.celsius

    var beamBroken = false

    var isSimulated = false

    override fun toLog(table: LogTable?) {
      table?.put("feederVelocity", feederVelocity.inRadiansPerSecond)
      table?.put("feederAppliedVoltage", feederAppliedVoltage.inVolts)
      table?.put("feederStatorCurrent", feederStatorCurrent.inAmperes)
      table?.put("feederSupplyCurrent", feederSupplyCurrent.inAmperes)
      table?.put("feederTempCelcius", feederTemp.inCelsius)
      table?.put("feederBeamBroken", beamBroken)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("feederVelocity", feederVelocity.inRadiansPerSecond)?.let {
        feederVelocity = it.radians.perSecond
      }

      table?.get("feederAppliedVoltage", feederAppliedVoltage.inVolts)?.let {
        feederAppliedVoltage = it.volts
      }

      table?.get("feederStatorCurrent", feederStatorCurrent.inAmperes)?.let {
        feederStatorCurrent = it.amps
      }

      table?.get("feederSupplyCurrent", feederSupplyCurrent.inAmperes)?.let {
        feederSupplyCurrent = it.amps
      }

      table?.get("feederTempCelsius", feederTemp.inCelsius)?.let { feederTemp = it.celsius }
      table?.get("feederBeamBroken", beamBroken)?.let { beamBroken = it }
    }
  }

  fun updateInputs(inputs: FeederIOInputs) {}

  fun setFeederVoltage(voltage: ElectricalPotential) {}

  fun setFeederBrakeMode(brake: Boolean) {}
}
