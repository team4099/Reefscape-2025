package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmConstants {
  val IDLE_CURRENT = 0.0.amps
  val RETRACT_CURRENT = 20.0.amps
  val EXTEND_CURRENT = -(20.0).amps
  val STALL_TIME_THRESHOLD = 0.15.seconds

  val ARM_TOLERANCE = 3.degrees
  val STATOR_CURRENT_LIMIT = 120.0.amps
  val SUPPLY_CURRENT_LIMIT = 120.0.amps

  val ARM_GEAR_RATIO = 10.0 / 48.0
  val SCORE_PROCESSOR_ANGLE = 40.degrees
  val EXTEND_ANGLE = 90.degrees
  val IDLE_ANGLE = 15.degrees
  val INTAKE_ALGAE_FROM_REEF_ANGLE = 40.degrees

  val INVERSION_VALUE: InvertedValue = InvertedValue.CounterClockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Brake

  val MAX_VELOCITY = 400.degrees.perSecond
  val MAX_ACCELERATION = 200.degrees.perSecond.perSecond

  val VOLTAGE_COMPENSATION = 12.0.volts

  object PID {
    val REAL_KP = 4.volts / 1.degrees
    val REAL_KI = 0.0.volts / (1.degrees * 1.seconds)
    val REAL_KD = 0.3.volts / (1.degrees.perSecond)
  }
}
