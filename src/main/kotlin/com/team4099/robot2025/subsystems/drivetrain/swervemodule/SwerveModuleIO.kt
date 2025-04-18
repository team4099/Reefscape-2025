package com.team4099.robot2025.subsystems.drivetrain.swervemodule

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface SwerveModuleIO {
  class SwerveModuleIOInputs : LoggableInputs {
    var driveAppliedVoltage = 0.0.volts
    var steeringAppliedVoltage = 0.0.volts

    var driveStatorCurrent = 0.0.amps
    var driveSupplyCurrent = 0.0.amps
    var steeringStatorCurrent = 0.0.amps
    var steeringSupplyCurrent = 0.0.amps

    var drivePosition = 0.0.meters
    var steeringPosition = 0.0.degrees

    var driveVelocity = 0.0.meters.perSecond
    var steeringVelocity = 0.0.degrees.perSecond

    var driveTemp = 0.0.celsius
    var steeringTemp = 0.0.celsius

    var odometryDrivePositions = listOf<Length>()
    var odometrySteeringPositions = listOf<Angle>()

    var potentiometerOutputRaw = 0.0
    var potentiometerOutputRadians = 0.0.radians

    var drift = 0.0.meters

    override fun toLog(table: LogTable?) {
      table?.put("driveAppliedVoltage", driveAppliedVoltage.inVolts)
      table?.put("steeringAppliedVoltage", steeringAppliedVoltage.inVolts)
      table?.put("driveStatorCurrentAmps", driveStatorCurrent.inAmperes)
      table?.put("driveSupplyCurrentAmps", driveSupplyCurrent.inAmperes)
      table?.put("steeringStatorCurrentAmps", steeringStatorCurrent.inAmperes)
      table?.put("steeringSupplyCurrentAmps", steeringSupplyCurrent.inAmperes)
      table?.put("drivePositionMeters", drivePosition.inMeters)
      table?.put("steeringPositionRadians", steeringPosition.inRadians)
      table?.put("driveVelocityMetersPerSecond", driveVelocity.inMetersPerSecond)
      table?.put("steeringVelocityRadiansPerSecond", steeringVelocity.inRadiansPerSecond)
      table?.put("driveTempCelcius", driveTemp.inCelsius)
      table?.put("steeringTempCelcius", steeringTemp.inCelsius)
      table?.put("potentiometerOutputRaw", potentiometerOutputRaw)
      table?.put("potentiometerOutputRadians", potentiometerOutputRadians.inRadians)
      table?.put("driftPositionMeters", drift.inMeters)

      if (odometryDrivePositions.size > 0) {
        table?.put("odometryDrivePositionsMeters", odometryDrivePositions[0].inMeters)
      } else {
        table?.put("odometryDrivePositionsMeters", 0.0)
      }
      if (odometrySteeringPositions.size > 0) {
        table?.put("odometrySteeringPositionsDegrees", odometrySteeringPositions[0].inDegrees)
      } else {
        table?.put("odometrySteeringPositionsDegrees", 0.0)
      }
    }

    override fun fromLog(table: LogTable?) {
      table?.get("driveAppliedVoltage", driveAppliedVoltage.inVolts)?.let {
        driveAppliedVoltage = it.volts
      }
      table?.get("steeringAppliedVoltage", steeringAppliedVoltage.inVolts)?.let {
        steeringAppliedVoltage = it.volts
      }
      table?.get("driveStatorCurrentAmps", driveStatorCurrent.inAmperes)?.let {
        driveStatorCurrent = it.amps
      }
      table?.get("driveSupplyCurrentAmps", driveSupplyCurrent.inAmperes)?.let {
        driveSupplyCurrent = it.amps
      }
      table?.get("steeringStatorCurrentAmps", steeringStatorCurrent.inAmperes)?.let {
        steeringStatorCurrent = it.amps
      }
      table?.get("steeringSupplyCurrentAmps", steeringSupplyCurrent.inAmperes)?.let {
        steeringSupplyCurrent = it.amps
      }
      table?.get("drivePositionMeters", drivePosition.inMeters)?.let { drivePosition = it.meters }
      table?.get("steeringPositionRadians", steeringPosition.inRadians)?.let {
        steeringPosition = it.radians
      }
      table?.get("driveVelocityMetersPerSecond", driveVelocity.inMetersPerSecond)?.let {
        driveVelocity = it.meters.perSecond
      }
      table?.get("steeringVelocityRadiansPerSecond", steeringVelocity.inRadiansPerSecond)?.let {
        steeringVelocity = it.radians.perSecond
      }
      table?.get("driveTempCelcius", driveTemp.inCelsius)?.let { driveTemp = it.celsius }
      table?.get("steeringTempCelcius", steeringTemp.inCelsius)?.let { steeringTemp = it.celsius }
      table?.get("potentiometerOutputRaw", potentiometerOutputRaw)?.let {
        potentiometerOutputRaw = it
      }
      table?.get("potentiometerOutputRaw", potentiometerOutputRadians.inRadians)?.let {
        potentiometerOutputRadians = it.radians
      }
      table?.get("driftPositionMeters", drift.inMeters)?.let { drift = it.meters }
    }
  }

  val label: String

  fun updateInputs(inputs: SwerveModuleIOInputs) {}

  fun setSteeringSetpoint(angle: Angle) {}
  fun setClosedLoop(steering: Angle, speed: LinearVelocity, acceleration: LinearAcceleration) {}
  fun setOpenLoop(steering: Angle, speed: LinearVelocity) {}
  fun runCharacterization(input: ElectricalPotential) {}

  fun resetModuleZero() {}
  fun zeroSteering(isInAutonomous: Boolean = false) {}
  fun zeroDrive() {}

  fun setDriveBrakeMode(brake: Boolean) {}

  fun setSteeringBrakeMode(brake: Boolean) {}

  fun configureDrivePID(
    kP: ProportionalGain<Velocity<Meter>, Volt>,
    kI: IntegralGain<Velocity<Meter>, Volt>,
    kD: DerivativeGain<Velocity<Meter>, Volt>,
    kV: Value<Fraction<Volt, Velocity<Meter>>>,
    kA: Value<Fraction<Volt, Velocity<Velocity<Meter>>>>
  ) {}
  fun configureSteeringPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}
  fun configureSteeringMotionMagic(maxVel: AngularVelocity, maxAccel: AngularAcceleration) {}
}
