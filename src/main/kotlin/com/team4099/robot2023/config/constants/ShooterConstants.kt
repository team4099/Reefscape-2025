package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

object ShooterConstants {
  // val ROLLER_GEAR_RATIO = 0.0
  // val ROLLER_VOLTAGE_COMPENSATION = 0.0.volts
  // val ROLLER_STATOR_CURRENT_LIMIT = 0.0.amps

  val ABSOLUTE_ENCODER_TO_MECHANISM_GEAR_RATIO = 0.0
  val WRIST_VOLTAGE_COMPENSATION = 0.0.volts
  val WRIST_STATOR_CURRENT_LIMIT = 0.0.amps

  // val FEEDER_GEAR_RATIO = 0.0
  // val FEEDER_VOLTAGE_COMPENSATION = 0.0.volts
  // val FEEDER_STATOR_CURRENT_LIMIT = 0.0.amps

  // val ROLLLER_INIT_VOLTAGE = 0.0.volts
  // val FEEDER_INIT_VOLTAGE = 0.0.volts
  val WRIST_INIT_VOLTAGE = 0.0.volts

  // val ROLLER_SHOOTING_VOLTAGE = 0.0.volts
  val WRIST_SOFTLIMIT_UPWARDSTURN = 0.0.degrees
  val WRIST_SOFTLIMIT_DOWNWARDSTURN = 0.0.degrees
}
