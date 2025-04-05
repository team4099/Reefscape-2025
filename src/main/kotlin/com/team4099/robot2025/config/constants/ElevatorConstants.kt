package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants {

  val TESTING_VOLTAGE = 0.0.volts
  val SLAM_VOLTAGE = -12.0.volts
  const val ENABLE_ELEVATOR = true

  val GEAR_RATIO = 16.0 / 48.0
  val CARRIAGE_MASS = 12.0.pounds

  val OPEN_LOOP_EXTEND_VOLTAGE = 8.0.volts
  val OPEN_LOOP_RETRACT_VOLTAGE = -(12.0.volts)
  val HOMING_APPLIED_VOLTAGE = -1.0.volts
  val HOMING_STALL_CURRENT = 15.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.15.seconds
  val UPWARDS_EXTENSION_LIMIT = 70.0.inches
  val DOWNWARDS_EXTENSION_LIMIT = -0.5.inches

  val SPOOL_DIAMETER = 2.0051.inches
  val VOLTAGE_COMPENSATION = 12.0.volts

  val ELEVATOR_TOLERANCE = 0.5.inches
  val ELEVATOR_FORCE_HOME_TOLERANCE = 1.inches
  val ELEVATION_EJECT_TOLERANCE = 0.125.inches

  val LEADER_STATOR_CURRENT_LIMIT = 60.0.amps
  val LEADER_SUPPLY_CURRENT_LIMIT = 60.0.amps
  val LEADER_SUPPLY_CURRENT_LOWER_LIMIT = 60.0.amps
  val LEADER_SUPPLY_CURRENT_LOWER_TIME = 60.0.amps

  val FOLLOWER_STATOR_CURRENT_LIMIT = 60.0.amps
  val FOLLOWER_SUPPLY_CURRENT_LIMIT = 60.0.amps
  val FOLLOWER_SUPPLY_CURRENT_LOWER_LIMIT = 60.0.amps
  val FOLLOWER_SUPPLY_CURRENT_LOWER_TIME = 60.0.amps

  // TODO: recheck
  /* thresholds for changing PID/FF values */
  val FIRST_STAGE_HEIGHT = 24.25.inches
  val SECOND_STAGE_HEIGHT = 49.5.inches

  val IDLE_HEIGHT = 0.0.inches
  val IDLE_CORAL_HORIZONTAL_HEIGHT = 0.0.inches
  val IDLE_CORAL_VERTICAL_HEIGHT = 0.0.inches
  val IDLE_ALGAE_HEIGHT = 0.0.inches

  val INTAKE_CORAL_HEIGHT = 0.inches

  val PASS_THROUGH_HEIGHT = 0.inches

  val INTAKE_ALGAE_GROUND_HEIGHT = 0.0.inches
  val INTAKE_ALGAE_L2_HEIGHT = 15.5.inches
  val INTAKE_ALGAE_L3_HEIGHT = 30.inches

  val SCORE_PROCESSOR_HEIGHT = 0.0.inches
  val SCORE_BARGE_HEIGHT = 72.5.inches

  // TODO: adjust scoring/intaking heights for end-effector offset
  val L1_HEIGHT = 24.inches
  val L2_HEIGHT = 28.789.inches - 2.inches + 0.5.inches + 5.5.inches + 1.0.inches
  val L3_HEIGHT = 28.789.inches - 2.inches + 0.5.inches + 20.5.inches + 0.5.inches
  val PREP_L4_HEIGHT = 53.5.inches
  val L4_HEIGHT = 72.5.inches

  val EJECT_HEIGHT = 9.0.inches

  val ELEVATOR_GROUND_OFFSET = 3.829.inches

  // TODO: make real maxes
  val MAX_VELOCITY = 300.0.inches.perSecond
  val MAX_ACCELERATION = 200.0.inches.perSecond.perSecond

  object PID {
    // TODO: tune all
    val REAL_KP = 5.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.3.volts / (1.inches.perSecond)

    val SIM_KP = 5.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)

    val SIM_KD = 0.0.volts / (1.inches.perSecond)

    val KS = 0.1.volts
    val KV = (0.32.volts) / 1.0.meters.perSecond //  0.037
    val KA = (0.03.volts) / 1.0.meters.perSecond.perSecond // 0.0025

    val KV_ADD = (0.1.volts) / 1.0.meters.perSecond //  0.037

    val KG_DEFAULT = 0.16.volts
    val KG_FIRST_STAGE = 0.16.volts
    val KG_SECOND_STAGE = 0.24.volts
    val KG_THIRD_STAGE = 0.32.volts
  }
}
