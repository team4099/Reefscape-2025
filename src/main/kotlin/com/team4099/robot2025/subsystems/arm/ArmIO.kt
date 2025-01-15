package com.team4099.robot2025.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.*
import org.team4099.lib.units.base.*
import org.team4099.lib.units.derived.*

interface ArmIO {

    class ArmIOInputs : LoggableInputs {
        // Arm Inputs
        var armPosition = 0.0.degrees
        var armVelocity = 0.0.degrees.perSecond
        var armAcceleration = 0.0.degrees.perSecond.perSecond
        var armAppliedVoltage = 0.0.volts
        var armStatorCurrent = 0.0.amps
        var armSupplyCurrent = 0.0.amps
        var armTemp = 0.0.celsius

        var isSimulating = false


        override fun toLog(table: LogTable?) {
            table?.put("armPositionDegrees", armPosition.inDegrees)
            table?.put("armVelocity", armVelocity.inDegreesPerSecond)
            table?.put("armAcceleration", armAcceleration.inDegreesPerSecondPerSecond)
            table?.put("armAppliedVoltage", armAppliedVoltage.inVolts)
            table?.put("armStatorCurrentAmps", armStatorCurrent.inAmperes)
            table?.put("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)
            table?.put("armTempCelsius", armTemp.inCelsius)
        }

        override fun fromLog(table: LogTable?) {

            table?.get("armPositionDegrees", armPosition.inDegrees)?.let {
                armPosition = it.degrees
            }
            table?.get("armVelocity", armVelocity.inDegreesPerSecond)?.let {
                armVelocity = it.rotations.perMinute
            }
            table?.get("armAcceleration", armAcceleration.inDegreesPerSecondPerSecond)?.let {
                armAcceleration = it.degrees.perSecond.perSecond
            }
            table?.get("armAppliedVoltage", armAppliedVoltage.inVolts)?.let {
                armAppliedVoltage = it.volts
            }
            table?.get("armStatorCurrentAmps", armStatorCurrent.inAmperes)?.let {
                armStatorCurrent = it.amps
            }
            table?.get("armSupplyCurrentAmps", armSupplyCurrent.inAmperes)?.let {
                armSupplyCurrent = it.amps
            }
            table?.get("armTempCelcius", armTemp.inCelsius)?.let {
               armTemp = it.celsius }
        }

    }

    fun updateInputs(inputs: ArmIOInputs) {}

    fun setArmVoltage(voltage: ElectricalPotential) {}

    fun setArmPosition(position: Angle) {}

    fun zeroEncoder() {}

    fun setArmBrakeMode(brake: Boolean) {}

    fun configurePID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {}

    fun configureFeedforward(
        kG: ElectricalPotential,
        kS: ElectricalPotential,
        kA: AccelerationFeedforward<Radian, Volt>,
        kV: VelocityFeedforward<Radian, Volt>
    )



}