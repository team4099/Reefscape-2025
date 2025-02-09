package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object RollersConstants {

  val STATOR_CURRENT_LIMIT = 40.0.amps
  val SUPPLY_CURRENT_LIMIT = 60.0.amps

  val INTAKE_VOLTAGE = 0.0.volts

  val IDLE_VOLTAGE = 0.0.volts
  val IDLE_CORAL_VOLTAGE = 0.0.volts
  val IDLE_ALGAE_VOLTAGE = 0.0.volts
  val INTAKE_ALGAE_VOLTAGE = 0.0.volts
  val INTAKE_CORAL_VOLTAGE = -2.5.volts

  val SCORE_BARGE_ALGAE_VOLTAGE = 0.0.volts
  val SCORE_PROCESSOR_ALGAE_VOLTAGE = 0.0.volts
  val SCORE_CORAL_VOLTAGE = 2.5.volts
  val EJECT_VOLTAGE = 0.0.volts

  val CORAL_VELOCITY_THRESHOLD = 0.0.rotations.perSecond
  val CORAL_CURRENT_THRESHOLD = 0.0.amps
  val ALGAE_VELOCITY_THRESHOLD = 0.0.rotations.perSecond
  val ALGAE_CURRENT_THRESHOLD = 0.0.amps

  val CORAL_DETECTION_TIME_THRESHOLD = 0.0.seconds
  val ALGAE_DETECTION_TIME_THRESHOLD = 0.0.seconds

  val CORAL_SPIT_TIME = 0.5.seconds
  val ALGAE_PROCESSOR_SPIT_TIME = 0.5.seconds
  val ALGAE_BARGE_SPIT_TIME = 0.0.seconds

  val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Coast

  val GEAR_RATIO = 24.0 / 12.0
  val VOLTAGE_COMPENSATION = 12.0.volts
  val INERTIA = 0.0010.kilo.grams * 1.0.meters.squared

  val BEAM_BREAK_FILTER_TIME = 0.05.seconds
}
