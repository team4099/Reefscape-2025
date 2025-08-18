package com.team4099.robot2025.subsystems.intake


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
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface IntakeIO { //blueprint for talon, and used within your logic class
    class IntakeIOInputs : LoggableInputs {
        //Creating intake inputs that we are tracking
        var intakeVelocity = 0.0.degrees.perSecond
        var intakeAcceleration = 0.0.degrees.perSecond.perSecond
        var intakeAppliedVoltage = 0.0.volts
        var intakeStatorCurrent = 0.0.amps
        var intakeSupplyCurrent = 0.0.amps
        var intakeTemp = 0.0.celsius
        var intakePosition = 0.0.degrees //needed for PID, and closed loop so we can track the exact position of our mech

        override fun toLog(table: LogTable?) {//imported from 6328 since we use ascope, logs intake's motor's inputs, and how they're changing
            table?.put("intakeVelocityDegreesPerSecond", intakeVelocity.inDegreesPerSecond) //Converts from our units (optimized, into a double)
            table?.put("intakeAccelerationDegreesPerSecondPerSecond", intakeAcceleration.inDegreesPerSecondPerSecond)
            table?.put("intakeAppliedVoltage", intakeAppliedVoltage.inVolts)
            table?.put("intakeStatorCurrent", intakeStatorCurrent.inAmperes)
            table?.put("intakeSupplyCurrent", intakeSupplyCurrent.inAmperes)
            table?.put("intakeTemp", intakeTemp.inCelsius)
            table?.put("intakePosition", intakePosition.inDegrees)
        }

        override fun fromLog(table: LogTable?) { //using let allows the table to convert in and out of our custom units, how exactly does that happen because of lambdas?
            table?.get("intakeVelocityDegreesPerSecond", intakeVelocity.inDegreesPerSecond)?.let {
                intakeVelocity = it.degrees.perSecond
            }
            table?.get("intakeAccelerationDegreesPerSecondPerSecond", intakeAcceleration.inDegreesPerSecondPerSecond)?.let {
                intakeAcceleration = it.degrees.perSecond.perSecond
            }
            table?.get("intakeAppliedVoltage", intakeAppliedVoltage.inVolts)?.let {
                intakeAppliedVoltage = it.volts
            }
            table?.get("intakeSupplyCurrent", intakeSupplyCurrent.inAmperes)?.let {
                intakeSupplyCurrent = it.amps
            }
            table?.get ("intakeStatorCurrent", intakeStatorCurrent.inAmperes)?.let {
                intakeStatorCurrent = it.amps
            }
            table?.get("intakeTemp", intakeTemp.inCelsius)?.let {
                intakeTemp = it.celsius
            }
            table?.get("intakePosition", intakePosition.inDegrees)?.let {
                intakePosition = it.degrees
            }
        }
        //Functions necessary for logic file.
    }
    fun configPID (
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {}


    fun configFF(
        kG: ElectricalPotential,
        kS: StaticFeedforward<Volt>,
        kV: VelocityFeedforward<Radian,Volt>,
        kA: AccelerationFeedforward<Radian, Volt>
    )

    fun updateInputs (inputs: IntakeIOInputs) //function that updates motor inputs
    fun setIntakeBrakeMode (brake: Boolean) //sets the motor's to have a brakemode
    fun setIntakeVoltage (volts: ElectricalPotential) //needed for voltage control, where you supply n volts continously to do smth
    fun setIntakePosition(position: Angle)
    fun zeroEncoder() {}
}