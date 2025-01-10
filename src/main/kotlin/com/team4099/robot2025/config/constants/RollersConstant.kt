package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps

object RollersConstant {

    val STATOR_CURRENT_LIMIT = 40.0.amps
    val SUPPLY_CURRENT_LIMIT = 120.0.amps


    val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
    val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Brake


}