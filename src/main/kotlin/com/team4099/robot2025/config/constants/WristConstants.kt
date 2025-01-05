package com.team4099.robot2023.config.constants

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
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object WristConstants {
  // val ROLLER_GEAR_RATIO = 0.0
  // val ROLLER_VOLTAGE_COMPENSATION = 0.0.volts
  // val ROLLER_STATOR_CURRENT_LIMIT = 0.0.amps
  val UNDER_STAGE_SHOT = 25.5.degrees
  val PUSH_DOWN_VOLTAGE = -0.5.volts

  val EJECT_ANGLE = -15.degrees
  val WRIST_AXIS_TO_NOTE_HOLD_POSITION = 14.5.inches
  val WRIST_AXIS_TO_NOTE_LAUNCH_POSITION = 10.inches

  val NOTE_ANGLE_SIM_OFFSET = -24.degrees

  val ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO = 69.0 / 20.0
  val MOTOR_TO_ABSOLUTE_ENCODER_GEAR_RATIO =
    5.0 / 1.0 * 4.0 / 1.0 * 54.0 / 18.0 * 48.0 / 18.0 * 1.0 / (69.0 / 20.0)

  val VOLTAGE_COMPENSATION = 12.0.volts
  val ABSOLUTE_ENCODER_OFFSET =
    (-34.5.degrees + 3.95.degrees) * ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO
  val WRIST_LENGTH = 18.6.inches
  val WRIST_INERTIA = 0.7181257183.kilo.grams * 1.0.meters.squared

  // val WRIST_GEAR_RATIO = 1.0 / 5.0 * 1.0 / 4.0 * 1.0 / 3.0 * 42.0 / 46.0 * 33.0 / 90.0

  val WRIST_VOLTAGE_COMPENSATION = 12.0.volts
  val WRIST_STATOR_CURRENT_LIMIT = 30.0.amps
  val WRIST_SUPPLY_CURRENT_LIMIT = 30.amps
  val WRIST_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val WRIST_TRIGGER_THRESHOLD_TIME = 10.0.seconds

  val WRIST_MAX_ROTATION = 20.degrees
  val WRIST_MIN_ROTATION = -36.25.degrees

  val WRIST_ZERO_SIM_OFFSET = 27.5.degrees

  val MAX_WRIST_VELOCITY = 400.degrees.perSecond
  val MAX_WRIST_ACCELERATION = 400.degrees.perSecond.perSecond

  val HARDSTOP_OFFSET = 47.degrees
  object PID {

    val ARBITRARY_FEEDFORWARD = 0.03.volts

    val REAL_KP: ProportionalGain<Radian, Volt> = 0.31.volts / 1.0.degrees
    val REAL_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val REAL_KD: DerivativeGain<Radian, Volt> = 0.05.volts / 1.0.degrees.perSecond

    val FIRST_STAGE_POS_SWITCH_THRESHOLD = 3.0.degrees
    val FIRST_STAGE_VEL_SWITCH_THRESHOLD = 3.0.degrees.perSecond

    val FIRST_STAGE_KP: ProportionalGain<Radian, Volt> = 0.35.volts / 1.0.degrees
    val FIRST_STAGE_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val FIRST_STAGE_KD: DerivativeGain<Radian, Volt> = 0.03.volts / 1.0.degrees.perSecond

    val SECOND_STAGE_POS_SWITCH_THRESHOLD = 0.8.degrees
    val SECOND_STAGE_VEL_SWITCH_THRESHOLD = 5.0.degrees.perSecond

    val SECOND_STAGE_KP: ProportionalGain<Radian, Volt> = 1.8.volts / 1.0.degrees
    val SECOND_STAGE_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val SECOND_STAGE_KD: DerivativeGain<Radian, Volt> = 0.075.volts / 1.0.degrees.perSecond

    val SIM_KP: ProportionalGain<Radian, Volt> = 10.volts / 1.0.degrees
    val SIM_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val SIM_KD: DerivativeGain<Radian, Volt> = 0.0175.volts / (1.0.degrees / 1.0.seconds)

    val REAL_WRIST_KG = 0.0.volts
    val REAL_WRIST_KV = 0.0.volts / 1.0.radians.perSecond
    val REAL_WRIST_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val REAL_WRIST_KS = 0.0.volts

    val SIM_WRIST_KG = 1.3.volts
    val SIM_WRIST_KV = 1.6.volts / 1.0.radians.perSecond
    val SIM_WRIST_KA = 0.175.volts / 1.0.radians.perSecond.perSecond
    val SIM_WRIST_KS = 0.15.volts
  }

  val WRIST_TOLERANCE = 0.3.degrees

  val IDLE_ANGLE = (-33.5).degrees

  val AMP_SCORE_ANGLE = -12.0.degrees
  val FAST_AMP_ANGLE = 27.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_LOW = -33.5.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_MID = 8.0.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_HIGH = -2.degrees
  val CLIMB_ANGLE = 0.0.degrees

  val TRAP_ANGLE = 35.degrees
  val INTAKE_ANGLE = (-33.25).degrees
  val IDLE_ANGLE_HAS_GAMEPEICE = -34.degrees
  val PASSING_SHOT_ANGLE = -34.degrees
}
