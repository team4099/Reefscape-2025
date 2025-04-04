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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ClimberConstants {
  // TODO: Change the constant values
  val VOLTAGE_COMPENSATION = 12.0.volts
  val STATOR_CURRENT_LIMIT = 30.0.amps
  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val THRESHOLD_CURRENT_LIMIT = 40.0.amps

  val CLIMB_EXTEND_VOLTAGE = -4.volts
  val CLIMB_RETRACT_VOLTAGE = 6.volts

  val LENGTH = 13.408.inches
  val INERTIA = 16.767866.pounds * 1.0.inches.squared
  val GEAR_RATIO = 240.0 / 1.0

  // TODO: Tune the PID values
  object PID {
    val KP_REAL: ProportionalGain<Radian, Volt> = 0.0.volts / 1.0.degrees
    val KI_REAL: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val KD_REAL: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.degrees.perSecond

    val KP_SIM = 0.2.volts / 1.0.degrees
    val KI_SIM = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val KD_SIM = 0.005.volts / 1.0.degrees.perSecond

    val KS = 0.0.volts
    val KV = 0.0.volts / 1.0.degrees.perSecond
    val KA = 0.0.volts / 1.0.degrees.perSecond.perSecond

    val KG_DEFAULT = 0.0.volts
    val KG_LATCHED = 0.0.volts
    val KG_UNLATCHED = 0.0.volts
  }
}
