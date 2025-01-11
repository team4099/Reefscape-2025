package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants {
  const val ENABLE_ELEVATOR = true

  val GEAR_RATIO = 16.0 / 48.0
  val CARRIAGE_MASS = 10.pounds

  val OPEN_LOOP_EXTEND_VOLTAGE = 8.0.volts
  val OPEN_LOOP_RETRACT_VOLTAGE = -(12.0.volts)
  val HOMING_APPLIED_VOLTAGE = 0.0.volts
  val HOMING_STALL_CURRENT = 0.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.0.seconds
  val UPWARDS_EXTENSION_LIMIT = 0.0.inches
  val DOWNWARDS_EXTENSION_LIMIT = 0.0.inches

  val ELEVATOR_PULLEY_TO_MOTOR = 0.0
  val SPOOL_DIAMETER = 0.0.inches
  val VOLTAGE_COMPENSATION = 12.0.volts

  val ELEVATOR_SAFE_THRESHOLD = 5.0.inches

  val ELEVATOR_TOLERANCE = 0.5.inches

  val LEADER_STATOR_CURRENT_LIMIT = 0.0
  val LEADER_SUPPLY_CURRENT_LIMIT = 0.0
  val LEADER_SUPPLY_CURRENT_LOWER_LIMIT = 0.0
  val LEADER_SUPPLY_CURRENT_LOWER_TIME = 0.0

  val FOLLOWER_STATOR_CURRENT_LIMIT = 0.0
  val FOLLOWER_SUPPLY_CURRENT_LIMIT = 0.0
  val FOLLOWER_SUPPLY_CURRENT_LOWER_LIMIT = 0.0
  val FOLLOWER_SUPPLY_CURRENT_LOWER_TIME = 0.0

  // change if incorrect
  val FIRST_STAGE_HEIGHT = 30.5.inches

  val L1_HEIGHT = 18.0.inches
  val L2_HEIGHT = 24.0.inches + (7.0 + 7.0 / 8.0).inches
  val L3_HEIGHT = 36.0.inches + (11.0 + 5.0 / 8.0).inches
  val L4_HEIGHT = 72.0.inches

  val ALGAE_LOW_HEIGHT = L2_HEIGHT
  val ALGAE_HIGH_HEIGHT = L3_HEIGHT

  val CORAL_STATION_HEIGHT = 36.0.inches + (1.0 + 1.0 / 2.0).inches

  val ELEVATOR_GROUND_OFFSET = 3.829.inches

  object PID {
    // TODO: tune all
    val REAL_KP = 8.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KP = 1.5.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.25.volts / (1.inches.perSecond)

    val SIM_ELEVATOR_KS_SECOND_STAGE = 0.0.volts
    val REAL_ELEVATOR_KS_SECOND_STAGE = 0.54.volts
    val ELEVATOR_KG_SECOND_STAGE = 1.0.volts
    val ELEVATOR_KV_SECOND_STAGE = 0.037.volts / 1.0.inches.perSecond
    val ELEVATOR_KA_SECOND_STAGE = 0.0025.volts / 1.0.inches.perSecond.perSecond

    val SIM_ELEVATOR_KS_FIRST_STAGE = 0.0.volts
    val REAL_ELEVATOR_KS_FIRST_STAGE = 0.54.volts
    val ELEVATOR_KG_FIRST_STAGE = 0.25.volts
    val ELEVATOR_KV_FIRST_STAGE = 0.037.volts / 1.0.inches.perSecond
    val ELEVATOR_KA_FIRST_STAGE = 0.0025.volts / 1.0.inches.perSecond.perSecond
  }
}
