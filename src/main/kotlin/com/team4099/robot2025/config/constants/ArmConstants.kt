package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmConstants {

  val ZERO_OFFSET = -24.09.degrees
  val ARM_GEAR_RATIO = 1 / 33.0
  val ARM_ENCODER_TO_MECHANISM_RATIO = 1.0

  val IDLE_ANGLE = 100.0.degrees
  val IDLE_CORAL_ANGLE = 100.0.degrees
  val IDLE_CORAL_L1_ANGLE = 100.degrees
  val IDLE_ALGAE_ANGLE = 0.0.degrees

  val SAFE_ELEVATOR_FRONT_ANGLE = 100.degrees
  val SAFE_ELEVATOR_BACK_ANGLE = 90.degrees

  val INTAKE_CORAL_ANGLE = -4.degrees
  // hotfix for now until encoder
  val INTAKE_L1_ANGLE = 100.degrees
  val INTAKE_ALGAE_GROUND_ANGLE = 205.degrees
  val INTAKE_ALGAE_L2_ANGLE = 120.0.degrees
  val INTAKE_ALGAE_L3_ANGLE = 120.0.degrees

  val SCORE_CORAL_L1_HORIZONTAL_ANGLE = 170.degrees
  val SCORE_CORAL_L1_VERTICAL_ANGLE = 120.degrees
  val SCORE_CORAL_L2_ANGLE = 100.degrees
  val SCORE_CORAL_L3_ANGLE = 100.degrees
  val SCORE_CORAL_L4_ANGLE = 100.degrees
  val SCORE_OFFSET = 20.degrees // how many degrees to go past to 'dunk' it

  val EJECT_ANGLE = 170.degrees

  val SCORE_ALGAE_PROCESSOR_ANGLE = 0.0.degrees
  val SCORE_ALGAE_BARGE_ANGLE = 90.0.degrees

  val TEST_ANGLE = 45.0.degrees

  // Feedforward Constants
  val ARM_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
  val ARM_KV = 0.0.volts / 1.0.radians.perSecond
  val ARM_KG = 0.15.volts
  val ARM_KS = 0.0.volts

  val STATOR_CURRENT_LIMIT = 30.0.amps
  val SUPPLY_CURRENT_LIMIT = 30.0.amps

  val INVERSION_VALUE: InvertedValue = InvertedValue.CounterClockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Coast

  val MOTION_MAGIC_ACCELERATION = 400.degrees.perSecond.perSecond
  val MOTION_MAGIC_CRUISE_VELOCITY = 500.degrees.perSecond

  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val ENCODER_DIRECTION_VALUE: SensorDirectionValue = SensorDirectionValue.Clockwise_Positive
  val ENCODER_OFFSET = 0.0.degrees

  val ARM_INERTIA = 16.767866.pounds * 1.0.inches.squared
  val ARM_LENGTH = 13.408.inches
  val ARM_MAX_ANGLE = 235.0.degrees
  val ARM_MIN_ANGLE = (-70.0).degrees
  val ARM_TOLERANCE = 2.0.degrees
  val LOOSE_ARM_TOLERANCE = 5.degrees

  val VOLTAGE_COMPENSATION = 12.0.volts

  object PID {
    // PID Constants
    val SIM_ARM_KP: ProportionalGain<Radian, Volt> = 0.15.volts / 1.0.degrees
    val SIM_ARM_KI: IntegralGain<Radian, Volt> = 0.01.volts / (1.0.degrees * 1.0.seconds)
    val SIM_ARM_KD: DerivativeGain<Radian, Volt> = 0.004.volts / 1.0.degrees.perSecond

    val REAL_ARM_KP: ProportionalGain<Radian, Volt> = 7.volts / 1.0.degrees
    val REAL_ARM_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val REAL_ARM_KD: DerivativeGain<Radian, Volt> = 0.3.volts / 1.0.degrees.perSecond
  }
}
