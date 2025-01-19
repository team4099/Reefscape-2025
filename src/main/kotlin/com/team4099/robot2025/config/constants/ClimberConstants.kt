package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond

object ClimberConstants {
    //TODO: Change the constant values
    val MAX_VELOCITY = 0.0.degrees.perSecond
    val MAX_ACCELERATION = 0.0.degrees.perSecond.perSecond
    val MAX_ANGLE = 0.0.degrees
    val MIN_ANGLE = 0.0.degrees
    val TOLERANCE = 0.0.degrees

    val HOMING_APPLIED_VOLTAGE = 0.0.volts
    val HOMING_STALL_CURRENT = 0.0.amps
    val HOMING_STALL_TIME_THRESHOLD = 0.0.seconds

    val VOLTAGE_COMPENSATION = 0.0.volts
    val STATOR_CURRENT_LIMIT = 0.0.amps
    val SUPPLY_CURRENT_LIMIT = 0.amps
    val THRESHOLD_CURRENT_LIMIT = 0.0.amps

    // TODO: Tune the PID values
    object PID {
        val KP_UNLATCH: ProportionalGain<Radian, Volt>  = 0.0.volts / 1.0.degrees
        val KI_UNLATCH: IntegralGain<Radian, Volt>  = 0.0.volts / (1.0.degrees * 1.0.seconds)
        val KD_UNLATCH: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.degrees.perSecond

        val KP_LATCH: ProportionalGain<Radian, Volt> = 0.0.volts / 1.0.degrees
        val KI_LATCH: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
        val KD_LATCH: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.degrees.perSecond

        val KP_SIM = 0.0.volts / 1.0.degrees
        val KI_SIM = 0.0.volts / (1.0.degrees * 1.0.seconds)
        val KD_SIM = 0.0.volts / 1.0.degrees.perSecond

        val KS_REAL = 0.0.volts
        val KV_REAL = 0.0.volts / 1.0.degrees.perSecond
        val KA_REAL = 0.0.volts / 1.0.degrees.perSecond.perSecond
        val KG_REAL = 0.0.volts

        val KS_SIM = 0.0.volts
        val KV_SIM = 0.0.volts / 1.0.degrees.perSecond
        val KA_SIM = 0.0.volts / 1.0.degrees.perSecond.perSecond
        val KG_SIM = 0.0.volts
    }
}