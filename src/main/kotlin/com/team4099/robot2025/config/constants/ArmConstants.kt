package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmConstants {

//  val IDLE_ANGLE = 100.0.degrees
//  val IDLE_CORAL_ANGLE = 100.0.degrees
//  val IDLE_CORAL_L1_ANGLE = 100.degrees
//  val IDLE_ALGAE_ANGLE = 0.0.degrees
//
//  val SAFE_ELEVATOR_FRONT_ANGLE = 100.degrees
//  val SAFE_ELEVATOR_BACK_ANGLE = 75.degrees
//
//  val INTAKE_CORAL_ANGLE = -2.75.degrees
//  // hotfix for now until encoder
//  val INTAKE_L1_ANGLE = 100.degrees
//  val INTAKE_ALGAE_GROUND_ANGLE = 205.degrees
//  val INTAKE_ALGAE_L2_ANGLE = 120.0.degrees
//  val INTAKE_ALGAE_L3_ANGLE = 120.0.degrees
//
//  val SCORE_CORAL_L1_HORIZONTAL_ANGLE = 170.degrees
//  val SCORE_CORAL_L1_VERTICAL_ANGLE = 120.degrees
//  val SCORE_CORAL_L2_ANGLE = 100.degrees
//  val SCORE_CORAL_L3_ANGLE = 100.degrees
//  val SCORE_CORAL_L4_ANGLE = 100.degrees
//  val SCORE_OFFSET = 20.degrees // how many degrees to go past to 'dunk' it
//
//  val EJECT_ANGLE = 170.degrees
//
//  val SCORE_ALGAE_PROCESSOR_ANGLE = 0.0.degrees
//  val SCORE_ALGAE_BARGE_ANGLE = 90.0.degrees
//
//  val TEST_ANGLE = 45.0.degrees

  val IDLE_VOLTAGE = 0.0.volts
  val HOMING_VOLTAGE = -1.0.volts
  val HOMING_STALL_CURRENT = 20.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.15.seconds
  val RETRACT_VOLTAGE = 12.0.volts
  val EXTEND_VOLTAGE = -12.0.volts

  val STATOR_CURRENT_LIMIT = 30.0.amps
  val SUPPLY_CURRENT_LIMIT = 30.0.amps

  val INVERSION_VALUE: InvertedValue = InvertedValue.CounterClockwise_Positive
  val NEUTRAL_MODE_VALUE: NeutralModeValue = NeutralModeValue.Coast


  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val VOLTAGE_COMPENSATION = 12.0.volts
}
