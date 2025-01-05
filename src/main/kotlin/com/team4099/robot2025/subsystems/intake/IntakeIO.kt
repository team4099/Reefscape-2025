package com.team4099.robot2023.subsystems.intake

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
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var rollerVelocity = 0.0.rotations.perMinute
    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    var centerWheelVelocity = 0.0.rotations.perMinute
    var centerWheelAppliedVotlage = 0.0.volts
    var centerWheelSupplyCurrent = 0.0.amps
    var centerWheelStatorCurrent = 0.0.amps
    var centerWheelTemp = 0.0.celsius

    var isSimulated = true

    override fun toLog(table: LogTable?) {
      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)
      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerTempCelsius", rollerTemp.inCelsius)

      table?.put("centerWheelVelocityRPM", centerWheelVelocity.inRotationsPerMinute)
      table?.put("centerWheelAppliedVoltage", centerWheelAppliedVotlage.inVolts)
      table?.put("centerWheelSupplyCurrentAmps", centerWheelSupplyCurrent.inAmperes)
      table?.put("centerWheelStatorCurrentAmps", centerWheelStatorCurrent.inAmperes)
      table?.put("centerWheelTempCelsius", centerWheelTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perSecond
      }

      table?.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }

      table?.get("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }

      table?.get("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }

      table?.get("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }

      table?.get("centerWheelVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        centerWheelVelocity = it.rotations.perSecond
      }

      table?.get("centerWheelAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        centerWheelAppliedVotlage = it.volts
      }

      table?.get("centerWheelSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        centerWheelSupplyCurrent = it.amps
      }

      table?.get("centerWheelStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        centerWheelStatorCurrent = it.amps
      }

      table?.get("centerWheelTempCelsius", centerWheelTemp.inCelsius)?.let {
        rollerTemp = it.celsius
      }
    }
  }

  fun updateInputs(io: IntakeIOInputs) {}

  fun setVoltage(rollerVoltage: ElectricalPotential, centerWheelVoltage: ElectricalPotential) {}

  fun setBrakeMode(rollerBrake: Boolean, centerWheelBrake: Boolean) {}
}
