package com.team4099.robot2023.config.constants

import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

object IntakeConstants {
  val ROLLER_INERTIA = 0.002459315 // this one has been updated
  val CENTER_WHEEL_INERTIA = 0.002459315
  val VOLTAGE_COMPENSATION = 12.0.volts

  val SIM_INTAKE_DISTANCE = 16.inches

  val INTAKE_TRANSFORM = Transform2d(Translation2d(-18.0.inches, 0.0.inches), 0.0.degrees)

  // TODO: Change gear ratio according to robot
  val ROLLER_CURRENT_LIMIT = 80.amps
  val ROLLER_SUPPLY_CURRENT_LIMIT = 120.0.amps
  val ROLLER_STATOR_CURRENT_LIMIT = 200.0.amps
  val ROLLER_CURRENT_TIME_THRESHOLD = 1.5.seconds
  val ROLLER_SUPPLY_TRIGGER_THRESHOLD = 65.amps
  const val ROLLER_MOTOR_INVERTED = true
  const val ROLLER_GEAR_RATIO = 24.0 / 12.0 // this one has been updated
  const val CENTER_WHEEL_GEAR_RATIO = 34.0 / 14.0

  // TODO: Update the idle roller voltage later
  val IDLE_ROLLER_VOLTAGE = 1.5.volts

  val IDLE_CENTER_WHEEL_VOLTAGE = 0.0.volts
  val INTAKE_ROLLER_VOLTAGE = -12.volts
  val INTAKE_CENTER_WHEEL_VOLTAGE = -12.volts
  val OUTTAKE_ROLLER_VOLTAGE = (10).volts
  val OUTTAKE_CENTER_WHEEL_VOLTAGE = (10).volts
}
