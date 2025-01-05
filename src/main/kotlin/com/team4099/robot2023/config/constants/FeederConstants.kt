package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object FeederConstants {

  val CLEAN_UP_TIME = 0.18.seconds
  val FEEDER_MOTOR_INVERTED = false
  val FEEDER_IDLE_VOLTAGE = 0.0.volts
  val VOLTAGE_COMPENSATION = 12.0.volts
  val FEEDER_CURRENT_LIMIT = 40.0.amps

  val BEAM_BREAK_WAIT_TIME = 0.3.seconds

  val FEEDER_GEAR_RATIO = 24.0 / 12.0
  val FEEDER_INERTIA = 0.0017672509.kilo.grams * 1.0.meters.squared

  var NOTE_VELOCITY_THRESHOLD = 60.degrees.perSecond

  var WAIT_BEFORE_DETECT_VELOCITY_DROP = 0.5.seconds

  val IDLE_VOLTAGE = 0.volts
  val INTAKE_NOTE_VOLTAGE = 5.0.volts
  val AUTO_INTAKE_NOTE_VOLTAGE = 1.5.volts

  val SHOOT_NOTE_VOLTAGE = 2.volts

  val OUTTAKE_NOTE_VOLTAGE = (-6).volts

  val beamBreakFilterTime = 0.05.seconds
}
