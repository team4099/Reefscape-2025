package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo

object RollersConstants {

  val STATOR_CURRENT_LIMIT = 40.0.amps
  val SUPPLY_CURRENT_LIMIT = 60.0.amps

  val INTAKE_VOLTAGE = 0.0.volts
  val SCORE_VOLTAGE = 0.0.volts

  val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Coast

  val GEAR_RATIO = 24.0 / 12.0
  val VOLTAGE_COMPENSATION = 12.0.volts
  val INERTIA = 0.0010.kilo.grams * 1.0.meters.squared
}
