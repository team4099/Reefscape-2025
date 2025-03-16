package com.team4099.robot2025.subsystems.arm

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface ArmIO {

  class ArmIOInputs : LoggableInputs {
    // Arm Inputs
    var armVelocity = 0.0.degrees.perSecond
    var armAcceleration = 0.0.degrees.perSecond.perSecond
    var armAppliedVoltage = 0.0.volts
    var armStatorCurrent = 0.0.amps
    var armSupplyCurrent = 0.0.amps
    var armTemp = 0.0.celsius

    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("armVelocityDegreesPerSecond", armVelocity.inDegreesPerSecond)
      table?.put(
        "armAccelerationDegreesPerSecondPerSecond", armAcceleration.inDegreesPerSecondPerSecond
      )
      table?.put("armAppliedVoltage", armAppliedVoltage.inVolts)
      table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)
      table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)
      table?.put("armTempCelsius", armTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {

      table?.get("armVelocityDegreesPerSecond", armVelocity.inDegreesPerSecond)?.let {
        armVelocity = it.degrees.perSecond
      }
      table?.get(
        "armAccelerationDegreesPerSecondPerSecond",
        armAcceleration.inDegreesPerSecondPerSecond
      )
        ?.let { armAcceleration = it.degrees.perSecond.perSecond }
      table?.get("armAppliedVoltage", armAppliedVoltage.inVolts)?.let {
        armAppliedVoltage = it.volts
      }
      table?.get("armStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
        armStatorCurrent = it.amps
      }
      table?.get("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
        armSupplyCurrent = it.amps
      }
      table?.get("armTempCelsius", armTemp.inCelsius)?.let { armTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: ArmIOInputs) {}

  fun setArmCurrent(amps: Current) {}

  fun setArmBrakeMode(brake: Boolean) {}
}
