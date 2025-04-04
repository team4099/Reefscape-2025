package com.team4099.robot2025.subsystems.climber

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inNewtons
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface ClimberIO {
  class ClimberInputs : LoggableInputs {
    var climberPosition = 0.0.degrees
    var climberVelocity = 0.0.degrees.perSecond
    var climberAcceleration = 0.0.degrees.perSecond.perSecond
    var climberTorque = 0.0.newtons
    var climberAppliedVoltage = 0.0.volts
    var climberDutyCycle = 0.0.volts
    var climberStatorCurrent = 0.0.amps
    var climberSupplyCurrent = 0.0.amps
    var climberTemperature = 0.0.celsius

    var isSimulated = false

    override fun toLog(table: LogTable) {
      table.put("climberPositionDegrees", climberPosition.inDegrees)
      table.put("climberVelocityDegreesPerSecond", climberVelocity.inDegreesPerSecond)
      table.put(
        "climberAccelerationDegreesPerSecondPerSecond",
        climberAcceleration.inDegreesPerSecondPerSecond
      )
      table.put("climberTorqueNewtonMeters", climberTorque.inNewtons)
      table.put("climberAppliedVolts", climberAppliedVoltage.inVolts)
      table.put("climberDutyCycleVolts", climberDutyCycle.inVolts)
      table.put("climberStatorCurrentAmps", climberStatorCurrent.inAmperes)
      table.put("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes)
      table.put("climberTemperatureCelsius", climberTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("climberPositionDegrees", climberPosition.inDegrees)?.let {
        climberPosition = it.degrees
      }

      table?.get("climberVelocityDegreesPerSecond", climberVelocity.inDegreesPerSecond)?.let {
        climberVelocity = it.degrees.perSecond
      }

      table?.get(
        "climberAccelerationDegreesPerSecondPerSecond",
        climberAcceleration.inDegreesPerSecondPerSecond
      )
        ?.let { climberAcceleration = it.degrees.perSecond.perSecond }

      table?.get("climberTorqueNewtonMeters", climberTorque.inNewtons)?.let {
        climberTorque = it.newtons
      }

      table?.get("climberAppliedVolts", climberAppliedVoltage.inVolts)?.let {
        climberAppliedVoltage = it.volts
      }
      table?.get("climberDutyCycleVolts", climberDutyCycle.inVolts)?.let {
        climberDutyCycle = it.volts
      }

      table?.get("climberStatorCurrentAmps", climberStatorCurrent.inAmperes)?.let {
        climberStatorCurrent = it.amps
      }
      table?.get("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes)?.let {
        climberSupplyCurrent = it.amps
      }

      table?.get("climberTemperatureCelsius", climberTemperature.inCelsius)?.let {
        climberTemperature = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ClimberInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setPosition(position: Angle, feedforward: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}

  fun zeroEncoder() {}

  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {}
}
