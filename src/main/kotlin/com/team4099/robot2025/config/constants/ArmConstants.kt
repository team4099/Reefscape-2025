package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ArmConstants {
  val IDLE_CURRENT = 0.0.amps
  val RETRACT_CURRENT = 20.0.amps
  val EXTEND_CURRENT = -(20.0).amps
  val STALL_TIME_THRESHOLD = 0.15.seconds

  val ENCODER_TO_MECHANISM_GEAR_RATIO = 20
  val INTAKE_MOMENT_OF_INERTIA = 12.kilo.grams.meterSquared
  val INTAKE_LENGTH = 5.meters
  val MIN_ROTATION = 0.degrees
  val MAX_ROTATION = 130.degrees

  val STATOR_CURRENT_LIMIT = 30.0.amps
  val SUPPLY_CURRENT_LIMIT = 30.0.amps

  val INVERSION_VALUE: InvertedValue = InvertedValue.CounterClockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Coast

  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val VOLTAGE_COMPENSATION = 12.0.volts
}
