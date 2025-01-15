package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ArmConstants {

  val ARM_SPROCKET_RATIO = (66.0 / 12.0)
  val ARM_MAX_PLANETARY_RATIO = (9.0 / 1.0)
  val ARM_GEAR_RATIO = ARM_SPROCKET_RATIO / ARM_MAX_PLANETARY_RATIO
  val ARM_ENCODER_TO_MECHANISM_RATIO = 1.0

  val IDLE_ANGLE = 0.0.degrees
  val IDLE_CORAL_ANGLE = 0.0.degrees
  val IDLE_ALGAE_ANGLE = 0.0.degrees

  val SAFE_ELEVATOR_FRONT_ANGLE = 0.0.degrees
  val SAFE_ELEVATOR_BACK_ANGLE = 0.0.degrees

  val INTAKE_CORAL_ANGLE = 0.0.degrees
  val INTAKE_ALGAE_GROUND_ANGLE = 0.0.degrees
  val INTAKE_ALGAE_L2_ANGLE = 0.0.degrees
  val INTAKE_ALGAE_L3_ANGLE = 0.0.degrees

  val SCORE_CORAL_ANGLED_ANGLE = 0.0.degrees
  val SCORE_CORAL_STRAIGHT_ANGLE = 0.0.degrees
  val SCORE_ALGAE_PROCESSOR_ANGLE = 0.0.degrees
  val SCORE_ALGAE_BARGE_ANGLE = 0.0.degrees

  // PID Constants
  val ARM_KP: ProportionalGain<Radian, Volt> = 0.35.volts / 1.0.degrees
  val ARM_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
  val ARM_KD: DerivativeGain<Radian, Volt> = 0.03.volts / 1.0.degrees.perSecond

  // Feedforward Constants
  val ARM_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
  val ARM_KV = 0.0.volts / 1.0.radians.perSecond
  val ARM_KG = 0.0.volts
  val ARM_KS = 0.0.volts

  val STATOR_CURRENT_LIMIT = 30.0.amps
  val SUPPLY_CURRENT_LIMIT = 30.0.amps

  val INVERSION_VALUE: InvertedValue = InvertedValue.Clockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Brake

  val MOTION_MAGIC_ACCELERATION = 100.0.degrees.perSecond.perSecond
  val MOTION_MAGIC_CRUISE_VELOCITY = 100.0.degrees.perSecond

  val ENCODER_DIRECTION_VALUE: SensorDirectionValue = SensorDirectionValue.Clockwise_Positive
  val ENCODER_OFFSET = 0.0.degrees

  val ARM_INERTIA = 10.kilo.grams * 1.0.meters.squared
  val ARM_LENGTH = 9.678.inches
  val ARM_MAX_ANGLE = 235.degrees
  val ARM_MIN_ANGLE = -70.degrees
  val ARM_TOLERANCE = 0.0.degrees

  val VOLTAGE_COMPENSATION = 12.volts
}
