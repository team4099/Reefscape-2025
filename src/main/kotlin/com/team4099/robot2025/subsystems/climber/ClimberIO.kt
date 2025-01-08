package com.team4099.robot2025.subsystems.climber

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface ClimberIO {
    class ClimberInputs: LoggableInputs {
        var climberPosition = 0.0.radians
        var climberVelocity = 0.0.radians.perSecond
        var climberAppliedVoltage = 0.0.volts
        var climberSupplyCurrent = 0.0.amps
        var climberStatorCurrent = 0.0.amps
        var climberTemperature = 0.0.celsius

        var isSimulated = false

        override fun toLog(table: LogTable) {
            table.put("climberPosition", climberPosition.inRadians)
            table.put("climberVelocity", climberVelocity.inRadiansPerSecond)
            table.put("climberAppliedVoltage", climberAppliedVoltage.inVolts)
            table.put("climberSupplyCurrent", climberSupplyCurrent.inAmperes)
            table.put("climberStatorCurrent", climberStatorCurrent.inAmperes)
            table.put("climberTemperature", climberTemperature.inCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("climberPositionRadians", climberPosition.inRadians)?.let { climberPosition = it.radians }
            table?.get("climberVelocityRadiansPerSecond", climberVelocity.inRadiansPerSecond)?.let { climberVelocity = it.radians.perSecond }
            table?.get("climberAppliedVolts", climberAppliedVoltage.inVolts)?.let { climberAppliedVoltage = it.volts }
            table?.get("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes)?.let { climberSupplyCurrent = it.amps }
            table?.get("climberStatorCurrentAmps", climberStatorCurrent.inAmperes)?.let { climberStatorCurrent = it.amps }
            table?.get("climberTemperatureCelsius", climberTemperature.inCelsius)?.let { climberTemperature = it.celsius }
        }
    }

    fun updateInputs(inputs: ClimberInputs) {}

    fun setClimberVoltage(voltage: ElectricalPotential) {}

    fun setFramePerimeterVoltage(voltage: ElectricalPotential) {}

    fun setClimberPosition(position: Angle, feedForward: ElectricalPotential, latched: Boolean) {}

    fun setFramePerimeterPosition(position: Angle, feedForward: ElectricalPotential, latched: Boolean) {}

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