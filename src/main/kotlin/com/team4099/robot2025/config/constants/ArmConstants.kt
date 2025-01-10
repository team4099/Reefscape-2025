package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmConstants {
    val ARM_TOLERANCE = 1.0.degrees

    val ARM_GEAR_RATIO = (48.0 / 1.0)
    val ARM_ENCODER_RATIO = 1.0

    val ARM_KP = 0.0.volts.perDegree
    val ARM_KI = 0.0.volts.perDegreeSeconds
    val ARM_KD = 0.0.volts / 1.degrees.perSecond

    val STATOR_CURRENT_LIMIT = 40.0.amps
    val SUPPLY_CURRENT_LIMIT = 120.0.amps

    val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
    val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Brake

    val MOTION_MAGIC_ACCELERATION = 1323.0.degrees.perSecond.perSecond
    val MOTION_MAGIC_CRUISE_VELOCITY = 18000.0.degrees.perSecond

    val FORWRARD_SOFT_LIMIT_THRESHOLD = 1331.degrees
    val REVERSE_SOFT_LIMIT_THRESHOLD = 1313.degrees

    val ENCODER_DIRECTION_VALUE: SensorDirectionValue = SensorDirectionValue.CounterClockwise_Positive
    val ENCODER_OFFSET = 0.0.degrees // TODO: Find correct units for this

    val GRAVITY_TYPE: GravityTypeValue = GravityTypeValue.Arm_Cosine
}