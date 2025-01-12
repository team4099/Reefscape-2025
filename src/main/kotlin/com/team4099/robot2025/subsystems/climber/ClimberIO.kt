package com.team4099.robot2025.subsystems.climber

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface ClimberIO {
    class ClimberInputs: LoggableInputs {
        var climberPosition = 0.0.radians
        var climberVelocity = 0.0.radians.perSecond
        var climberAcceleration = 0.0.radians.perSecond.perSecond
        var climberTorque = 0.0
        var climberAppliedVoltage = 0.0.volts
        var climberDutyCycle = 0.0.volts
        var climberStatorCurrent = 0.0.amps
        var climberSupplyCurrent = 0.0.amps
        var climberTemperature = 0.0.celsius

        var isSimulated = false

        override fun toLog(table: LogTable) {
            table.put("climberPosition", climberPosition.inRadians)
            table.put("climberVelocity", climberVelocity.inRadiansPerSecond)
            table.put("climberAcceleration", climberAcceleration.inRadiansPerSecondPerSecond)
            table.put("climberTorque", climberTorque)
            table.put("climberAppliedVoltage", climberAppliedVoltage.inVolts)
            table.put("climberDutyCycle", climberDutyCycle.inVolts)
            table.put("climberStatorCurrent", climberStatorCurrent.inAmperes)
            table.put("climberSupplyCurrent", climberSupplyCurrent.inAmperes)
            table.put("climberTemperature", climberTemperature.inCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("climberPositionRadians", climberPosition.inRadians)?.let { climberPosition = it.radians }

            table?.get("climberVelocityRadiansPerSecond", climberVelocity.inRadiansPerSecond)?.let {
                climberVelocity = it.radians.perSecond
            }

            table?.get(
                "climberAccelerationRadiansPerSecondPerSecond",
                climberAcceleration.inRadiansPerSecondPerSecond
            )?.let {
                climberAcceleration = it.radians.perSecond.perSecond
            }

            table?.get("climberTorque", climberTorque)?.let { climberTorque = it }
            table?.get("climberAppliedVolts", climberAppliedVoltage.inVolts)?.let { climberAppliedVoltage = it.volts }
            table?.get("climberDutyCycleVolts", climberDutyCycle.inVolts)?.let { climberDutyCycle = it.volts }
            table?.get("climberStatorCurrentAmps", climberStatorCurrent.inAmperes)?.let { climberStatorCurrent = it.amps }
            table?.get("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes)?.let { climberSupplyCurrent = it.amps }
            table?.get("climberTemperatureCelsius", climberTemperature.inCelsius)?.let { climberTemperature = it.celsius }
        }
    }

    fun updateInputs(inputs: ClimberInputs) {}

    fun setVoltage(voltage: ElectricalPotential) {}

    fun setPosition(position: Angle, feedforward: ElectricalPotential, latched: Boolean) {}

    fun zeroEncoder() {}

    fun configPID (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {}

    fun configPIDSlot1 (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {}

    fun configPIDSlot2 (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {}
}