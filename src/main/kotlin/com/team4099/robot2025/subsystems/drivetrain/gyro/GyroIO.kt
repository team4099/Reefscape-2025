package com.team4099.robot2025.subsystems.drivetrain.gyro

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface GyroIO {
  class GyroIOInputs : LoggableInputs {
    var rawGyroYaw = 0.0.radians
    var gyroYaw = 0.0.radians
    var gyroPitch = 0.0.radians
    var gyroRoll = 0.0.radians
    var gyroYawRate = 0.0.radians.perSecond
    var gyroPitchRate = 0.0.radians.perSecond
    var gyroRollRate = 0.0.radians.perSecond

    var odometryYawPositions = mutableListOf<Angle>()

    var gyroConnected = false

    override fun toLog(table: LogTable?) {
      table?.put("rawGyroYawDegrees", rawGyroYaw.inDegrees)
      table?.put("gyroYawAngleDegrees", gyroYaw.inDegrees)
      table?.put("gyroPitchAngleDegrees", gyroPitch.inDegrees)
      table?.put("gyroRollAngleDegrees", gyroRoll.inDegrees)
      table?.put("gyroYawRateDegreesPerSecond", gyroYawRate.inDegreesPerSecond)
      table?.put("gyroPitchRateDegreesPerSecond", gyroPitchRate.inDegreesPerSecond)
      table?.put("gyroRollRateDegreesPerSecond", gyroRollRate.inDegreesPerSecond)
      table?.put("gyroConnected", gyroConnected)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("rawGyroYawDegrees", rawGyroYaw.inDegrees)?.let { rawGyroYaw = it.degrees }
      table?.get("gyroYawAngleDegrees", gyroYaw.inDegrees)?.let { gyroYaw = it.degrees }
      table?.get("gyroPitchDegrees", gyroPitch.inDegrees)?.let { gyroPitch = it.degrees }
      table?.get("gyroRollDegrees", gyroRoll.inDegrees)?.let { gyroRoll = it.degrees }
      table?.get("gyroYawRateDegreesPerSecond", gyroYawRate.inDegreesPerSecond)?.let {
        gyroYawRate = it.degrees.perSecond
      }
      table?.get("gyroPitchRateDegreesPerSecond", gyroPitchRate.inDegreesPerSecond)?.let {
        gyroPitchRate = it.degrees.perSecond
      }
      table?.get("gyroRollRateDegreesPerSecond", gyroRollRate.inDegreesPerSecond)?.let {
        gyroRollRate = it.degrees.perSecond
      }
      table?.get("gyroConnected", gyroConnected)?.let { gyroConnected = it }
    }
  }
  fun updateInputs(inputs: GyroIOInputs) {}

  fun zeroGyroYaw(toAngle: Angle) {}

  fun zeroGyroPitch(toAngle: Angle) {}

  fun zeroGyroRoll(toAngle: Angle) {}
}
