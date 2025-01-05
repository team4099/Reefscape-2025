package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

object SuperstructureConstants {
  val distanceFlywheelSpeedTableReal =
    listOf(
      Pair(
        FieldConstants.subwooferX + FieldConstants.edgeOfBumperToCenter,
        3000.rotations.perMinute
      ),
      Pair(55.55.inches, 2500.rotations.perMinute),
      Pair(63.89.inches, 2500.rotations.perMinute),
      Pair(75.7.inches, 2500.rotations.perMinute),
      Pair(84.25.inches, 2500.rotations.perMinute),
      Pair(92.6.inches, 2500.rotations.perMinute),
      Pair(103.7.inches, 2500.rotations.perMinute),
      Pair(113.2.inches, 2500.rotations.perMinute),
      Pair(122.1.inches, 3000.rotations.perMinute),
      Pair(134.inches, 3000.rotations.perMinute),
      Pair(144.8.inches, 3000.rotations.perMinute),
      Pair(156.8.inches, 3500.rotations.perMinute),
      Pair(172.2.inches, 3500.rotations.perMinute),
      Pair(185.6.inches, 4000.rotations.perMinute),
      Pair(198.15.inches, 4000.rotations.perMinute),
      Pair(220.1.inches, 4400.rotations.perMinute)
    )

  val distanceWristAngleTableReal =
    listOf(
      Pair(FieldConstants.subwooferX + FieldConstants.edgeOfBumperToCenter, -34.5.degrees),
      Pair(55.55.inches, -30.degrees),
      Pair(63.89.inches, -25.degrees),
      Pair(75.5.inches, -20.degrees),
      Pair(84.25.inches, -17.degrees),
      Pair(92.6.inches, -15.25.degrees),
      Pair(103.7.inches, -13.degrees),
      Pair(113.23.inches, -11.degrees),
      Pair(122.0.inches, -10.5.degrees),
      Pair(134.inches, -9.degrees),
      Pair(143.8.inches, -6.75.degrees),
      Pair(156.8.inches, -5.5.degrees),
      Pair(166.9.inches, -4.5.degrees),
      Pair(172.2.inches, -4.375.degrees),
      Pair(185.6.inches, -2.95.degrees),
      Pair(198.15.inches, -2.2.degrees),
      Pair(220.1.inches, -2.degrees)
    )

  val highDistanceFlywheelSpeedTableReal =
    listOf(
      Pair(71.2.inches, 2500.rotations.perMinute),
      Pair(82.0.inches, 2500.rotations.perMinute),
      Pair(95.23.inches, 2500.rotations.perMinute),
      Pair(108.7.inches, 2500.rotations.perMinute),
      Pair(119.0.inches, 2500.rotations.perMinute),
      Pair(133.5.inches, 3000.rotations.perMinute),
      Pair(143.0.inches, 3000.rotations.perMinute),
      Pair(156.6.inches, 3500.rotations.perMinute)
    )

  val highDistanceWristAngleTableReal =
    listOf(
      Pair(71.2.inches, -10.degrees),
      Pair(82.0.inches, -7.25.degrees),
      Pair(95.23.inches, -5.5.degrees),
      Pair(108.7.inches, -5.degrees),
      Pair(119.0.inches, -4.25.degrees),
      Pair(133.5.inches, -3.65.degrees),
      Pair(143.0.inches, -1.5.degrees),
      Pair(156.6.inches, 0.degrees),
    )

  val distanceFlywheelSpeedTableSim =
    listOf(
      Pair(1.meters, 3000.rotations.perMinute),
      Pair(2.meters, 3500.rotations.perMinute),
      Pair(3.meters, 4000.rotations.perMinute),
      Pair(4.meters, 4500.rotations.perMinute),
      Pair(5.meters, 5000.rotations.perMinute),
      Pair(6.meters, 5500.rotations.perMinute),
      Pair(7.meters, 6000.rotations.perMinute),
      Pair(500.inches, 8000.rotations.perMinute)
    )

  val distanceWristAngleTableSim =
    listOf(
      Pair(46.5.inches, -35.degrees),
      Pair(75.inches, -22.degrees),
      Pair(117.inches, -11.5.degrees),
      Pair(149.inches, -6.5.degrees),
      Pair(191.inches, -2.degrees),
      Pair(253.inches, 1.5.degrees),
      Pair(321.inches, 4.degrees),
      Pair(500.inches, 10.degrees)
    )
}
