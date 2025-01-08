package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.base.seconds
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

    // TODO: Tune the PID values
    object PID {
        val KP_REAL = 0.0.volts / 1.0.radians
        val KI_REAL = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_REAL = 0.0.volts / 1.0.radians.perSecond

        val KP_SIM = 0.0.volts / 1.0.radians
        val KI_SIM = 0.0.volts / (1.0.radians * 1.0.seconds)
        val KD_SIM = 0.0.volts / 1.0.radians.perSecond

        val KS = 0.0.volts
        val KV = 0.0.volts / 1.0.radians.perSecond
        val KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    }
}