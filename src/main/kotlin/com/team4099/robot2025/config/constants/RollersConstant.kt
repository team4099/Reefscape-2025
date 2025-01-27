package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.kilo

object RollersConstant {

  val STATOR_CURRENT_LIMIT = 40.0.amps
  val SUPPLY_CURRENT_LIMIT = 120.0.amps

  val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Brake

  const val GEAR_RATIO = 10.0
  const val MAX_VOLTAGE = 12.0
  val INERTIA = 0.0010.kilo.grams * 1.0.meters.squared // put it to somethig random get actual ones
}
