package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond

object ClimberConstants {
    //TODO: Change the constant values
    val MAX_VELOCITY = 0.0.radians.perSecond
    val MAX_ACCELERATION = 0.0.radians.perSecond.perSecond
    val MAX_ANGLE = 0.0.radians
    val MIN_ANGLE = 0.0.radians
    val TOLERANCE = 0.0.radians

    val HOMING_APPLIED_VOLTAGE = 0.0.volts
    val HOMING_STALL_CURRENT = 0.0.amps
    val HOMING_STALL_TIME_THRESHOLD = 0.0.seconds

    val VOLTAGE_COMPENSATION = 0.0.volts
    val STATOR_CURRENT_LIMIT = 0.0.amps
    val SUPPLY_CURRENT_LIMIT = 0.amps
    val THRESHOLD_CURRENT_LIMIT = 0.0.amps

    // TODO: Tune the PID values
    object PID {
        val KP_REAL: ProportionalGain<Radian, Volt>  = 0.0.volts / 1.0.radians
        val KI_REAL: IntegralGain<Radian, Volt>  = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_REAL: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.radians.perSecond

        val KP_UNLATCH: ProportionalGain<Radian, Volt>  = 0.0.volts / 1.0.radians
        val KI_UNLATCH: IntegralGain<Radian, Volt>  = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_UNLATCH: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.radians.perSecond

        val KP_LATCH: ProportionalGain<Radian, Volt> = 0.0.volts / 1.0.radians
        val KI_LATCH: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_LATCH: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.radians.perSecond

        val KP_SIM = 0.0.volts / 1.0.radians
        val KI_SIM = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_SIM = 0.0.volts / 1.0.radians.perSecond

        val KS = 0.0.volts
        val KV = 0.0.volts / 1.0.radians.perSecond
        val KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    }
}