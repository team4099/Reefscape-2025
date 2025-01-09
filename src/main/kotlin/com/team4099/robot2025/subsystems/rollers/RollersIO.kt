package com.team4099.robot2025.subsystems.rollers

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.ElectricalPotential
interface RollersIO {
    class RollersIOInputs : LoggableInputs {
        // Roller Inputs
        var rollerVelocity = 0.rotations.perMinute
        var rollerAppliedVoltage = 0.0.volts
        var rollerStatorCurrent = 0.0.amps
        var rollerSupplyCurrent = 0.0.amps
        var rollerTemp = 0.0.celsius
        override fun toLog(table: LogTable?) {
            table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)
            table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
            table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
            table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
            table?.put("rollerTempCelcius", rollerTemp.inCelsius)

        }

        override fun fromLog(table: LogTable?) {
            table?.get("rollerVelocity", rollerVelocity.inRotationsPerMinute)?.let {
                rollerVelocity = it.rotations.perMinute
            }
            table?.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
                rollerAppliedVoltage = it.volts
            }
            table?.get("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
                rollerStatorCurrent = it.amps
            }
            table?.get("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
                rollerSupplyCurrent = it.amps
            }
        }

    }
    fun setRollerSpeed(speed: Double) {}

    fun setRollerVoltage(voltage: ElectricalPotential) {}

    fun setRollerBrakeMode(brake: Boolean) {}