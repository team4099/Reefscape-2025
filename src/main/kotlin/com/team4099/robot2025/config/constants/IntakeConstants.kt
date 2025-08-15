package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
    val VOLTAGE_COMPENSATION = 12.0.volts
    val STATOR_CURRENT_LIMIT = 30.0.amps
    val SUPPLY_CURRENT_LIMIT = 40.0.amps
    val THRESHOLD_CURRENT_LIMIT = 40.0.amps


    val LENGTH = 13.408.inches
    val INERTIA = 16.767866.pounds * 1.0.inches.squared
    val GEAR_RATIO = 240.0 / 1.0
    val INTAKE_TOLERANCE = 1.0.degrees


    // Feedforward Constants
    val INTAKE_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val INTAKE_KV = 0.0.volts / 1.0.radians.perSecond
    val INTAKE_KG = 0.15.volts
    val INTAKE_KS = 0.0.volts

    object PID {
        // PID Constants
        val SIM_PIVOT_KP: ProportionalGain<Radian, Volt> = 0.15.volts / 1.0.degrees
        val SIM_PIVOT_KI: IntegralGain<Radian, Volt> = 0.01.volts / (1.0.degrees * 1.0.seconds)
        val SIM_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.004.volts / 1.0.degrees.perSecond

        val REAL_INTAKE_KP: ProportionalGain<Radian, Volt> = 7.volts / 1.0.degrees
        val REAL_INTAKE_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
        val REAL_INTAKE_KD: DerivativeGain<Radian, Volt> = 0.3.volts / 1.0.degrees.perSecond
    }
}