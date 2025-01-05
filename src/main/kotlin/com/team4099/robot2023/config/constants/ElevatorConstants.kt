package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants {
  // TODO: Change values later based on CAD
  val REAL_KP = 8.0.volts / 1.inches
  val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
  val REAL_KD = 0.0.volts / (1.inches.perSecond)

  val CARRIAGE_MASS = 30.892.pounds

  val ELEVATOR_MAX_RETRACTION = 0.0.inches
  val ELEVATOR_MAX_EXTENSION = 16.0.inches
  val ELEVATOR_CLIMB_EXTENSION = 15.5.inches

  val LEADER_INVERTED = false
  val FOLLOWER_INVERTED = true

  val LEADER_KP: ProportionalGain<Meter, Volt> = 0.725.volts / 1.inches
  val LEADER_KI: IntegralGain<Meter, Volt> = 0.0.volts / (1.inches * 1.seconds)
  val LEADER_KD: DerivativeGain<Meter, Volt> = 0.0.volts / (1.inches.perSecond)

  val FOLLOWER_KP: ProportionalGain<Meter, Volt> = 0.725.volts / 1.inches
  val FOLLOWER_KI: IntegralGain<Meter, Volt> = 0.0.volts / (1.inches * 1.seconds)
  val FOLLOWER_KD: DerivativeGain<Meter, Volt> = 0.0.volts / (1.inches.perSecond)

  val SIM_KP = 1.volts / 1.inches
  val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
  val SIM_KD = 0.0.volts / (1.inches.perSecond)

  val ELEVATOR_KS = 0.0.volts
  val ELEVATOR_KG = 0.28.volts
  val ELEVATOR_KV = 1.0.volts / 1.inches.perSecond
  val ELEVATOR_KA = 0.075.volts / 1.inches.perSecond.perSecond
  val ELEVATOR_OPEN_LOOP_EXTEND_VOLTAGE = 8.0.volts
  val ELEVATOR_OPEN_LOOP_RETRACT_VOLTAGE = -12.0.volts

  val ENABLE_ELEVATOR = true
  val ELEVATOR_IDLE_HEIGHT = 0.0.inches
  val ELEVATOR_SOFT_LIMIT_EXTENSION = 16.0.inches
  val ELEVATOR_SOFT_LIMIT_RETRACTION = 0.0.inches
  val ELEVATOR_OPEN_LOOP_SOFT_LIMIT_EXTENSION = 0.0.inches
  val ELEVATOR_OPEN_LOOP_SOFT_LIMIT_RETRACTION = 0.0.inches
  val ELEVATOR_SAFE_THRESHOLD = 5.0.inches

  val ELEVATOR_TOLERANCE = 0.5.inches

  val MAX_VELOCITY = 2.meters.perSecond
  val MAX_ACCELERATION = 4.0.meters.perSecond.perSecond

  val SHOOT_SPEAKER_LOW_POSITION = 0.0.inches
  val SHOOT_SPEAKER_MID_POSITION = 16.0.inches
  val SHOOT_SPEAKER_HIGH_POSITION = 16.0.inches

  // week 1 amp amgle
  // val SHOOT_AMP_POSITION = 17.0.degrees
  val SHOOT_AMP_SHOOTER_SIDE_POSITION = 12.inches
  val SHOOT_AMP_FEEDER_SIDE_POSITION = 15.0.inches
  val SOURCE_NOTE_OFFSET = 0.0.inches
  val ELEVATOR_THETA_POS = 0.0.degrees
  val HOMING_STATOR_CURRENT = 3.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.2.seconds
  val HOMING_APPLIED_VOLTAGE = -1.volts
  val ELEVATOR_GROUND_OFFSET = 0.0.inches

  val VOLTAGE_COMPENSATION = 12.0.volts
  val ELEVATOR_PULLEY_TO_MOTOR = 1 / 4.0 * 1 / 4.0
  val SPOOL_DIAMETER = 1.591.inches

  val LEADER_SUPPLY_CURRENT_LIMIT = 0.0.amps
  val LEADER_THRESHOLD_CURRENT_LIMIT = 0.0.amps
  val LEADER_SUPPLY_TIME_THRESHOLD = 0.0.seconds
  val LEADER_STATOR_CURRENT_LIMIT = 30.0.amps

  val FOLLOWER_SUPPLY_TIME_THRESHOLD = 0.0.seconds
  val FOLLOWER_STATOR_CURRENT_LIMIT = 0.0.amps
  val FOLLOWER_SUPPLY_CURRENT_LIMIT = 0.0.amps
  val FOLLOWER_THRESHOLD_CURRENT_LIMIT = 0.0.amps
}
