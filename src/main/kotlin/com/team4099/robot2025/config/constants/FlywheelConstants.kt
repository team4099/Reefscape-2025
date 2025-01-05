package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FlywheelConstants {
  val LEFT_GEAR_RATIO = 1.0

  val FLYWHEEL_RADIUS = 2.inches

  val FLYWHEEL_SPEED_TRANSFER_PERCENTAGE = 0.47

  val RIGHT_MOTOR_REVOLUTIONS_PER_FLYWHEEL_REVOLUTIONS = 2.0

  val VOLTAGE_COMPENSATION = 12.volts

  val INERTIA = 0.0014550597.kilo.grams * 1.0.meters.squared

  val RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 50.0.amps
  val RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val RIGHT_flywheel_TRIGGER_THRESHOLD_TIME = 10.0.seconds
  val RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT = 50.0.amps

  val LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60.0.amps
  val LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val LEFT_flywheel_TRIGGER_THRESHOLD_TIME = 10.0.seconds
  val LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT = 50.0.amps

  val FLYWHEEL_TOLERANCE = 75.0.rotations.perMinute
  object PID {
    val REAL_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.00.volts / 1.0.rotations.perMinute
    val REAL_KI: IntegralGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val REAL_KD: DerivativeGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute.perSecond)

    val SIM_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.015.volts / 1.0.rotations.perMinute
    val SIM_KI: IntegralGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val SIM_KD: DerivativeGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute.perSecond)

    val REAL_FLYWHEEL_KS = 0.0.volts
    val REAL_FLYWHEEL_KV = 0.245.volts / 1.radians.perSecond
    val REAL_FLYWHEEL_KA = 1.8.volts / 1.radians.perSecond.perSecond

    val SIM_FLYWHEEL_KS = 0.0.volts
    val SIM_FLYWHEEL_KV = 0.1.volts / 1.radians.perSecond
    val SIM_FLYWHEEL_KA = 0.1.volts / 1.radians.perSecond.perSecond
  }

  val IDLE_VELOCITY = 0.0.rotations.perMinute
  val SPEAKER_VELOCITY = 2500.rotations.perMinute
  val PASSING_SHOT_VELOCITY = 2_000.rotations.perMinute

  val UNDER_STAGE_SHOT_VELOCITY = 2_500.rotations.perMinute

  val AMP_VELOCITY = 1_500.rotations.perMinute
  val TRAP_VELOCITY = 3_000.rotations.perMinute

  val AMP_SCORE_TIME = 0.2.seconds
  val SPEAKER_SCORE_TIME = 0.2.seconds
  val EJECT_VELOCITY = 3_000.rotations.perMinute
}
