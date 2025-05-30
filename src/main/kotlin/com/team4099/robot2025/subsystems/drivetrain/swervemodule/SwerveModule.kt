package com.team4099.robot2025.subsystems.drivetrain.swervemodule

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterPerSecondPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.withSign

class SwerveModule(val io: SwerveModuleIO) {
  val inputs = SwerveModuleIO.SwerveModuleIOInputs()

  var modulePosition = SwerveModulePosition()

  var positionDeltas = mutableListOf<SwerveModulePosition>()

  private var speedSetPoint: LinearVelocity = 0.feet.perSecond
  private var accelerationSetPoint: LinearAcceleration = 0.feet.perSecond.perSecond

  private var steeringSetPoint: Angle = 0.degrees

  private var lastDrivePosition = 0.meters

  private var shouldInvert = false

  private val driveKV =
    LoggedTunableValue(
      "Drivetrain/kV",
      DrivetrainConstants.PID.DRIVE_KV,
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )
  private val driveKA =
    LoggedTunableValue(
      "Drivetrain/kA",
      DrivetrainConstants.PID.DRIVE_KA,
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  private val steeringkP =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree })
    )
  private val steeringkI =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkI",
      Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val steeringkD =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringkD",
      Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  private val steeringMaxVel =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringMaxVelRadPerSec", DrivetrainConstants.STEERING_VEL_MAX
    )
  private val steeringMaxAccel =
    LoggedTunableValue(
      "Drivetrain/moduleSteeringMaxAccelRadPerSecSq", DrivetrainConstants.STEERING_ACCEL_MAX
    )

  private val drivekP =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekP",
      Pair({ it.inVoltsPerMetersPerSecond }, { it.volts.perMeterPerSecond })
    )

  private val drivekI =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekI",
      Pair({ it.inVoltsPerMeters }, { it.volts / (1.meters.perSecond * 1.seconds) })
    )

  private val drivekD =
    LoggedTunableValue(
      "Drivetrain/moduleDrivekD",
      Pair({ it.inVoltsPerMetersPerSecondPerSecond }, { it.volts.perMeterPerSecondPerSecond })
    )

  init {
    if (isReal()) {
      steeringkP.initDefault(DrivetrainConstants.PID.STEERING_KP)
      steeringkI.initDefault(DrivetrainConstants.PID.STEERING_KI)
      steeringkD.initDefault(DrivetrainConstants.PID.STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.DRIVE_KD)
    } else {
      steeringkP.initDefault(DrivetrainConstants.PID.SIM_STEERING_KP)
      steeringkI.initDefault(DrivetrainConstants.PID.SIM_STEERING_KI)
      steeringkD.initDefault(DrivetrainConstants.PID.SIM_STEERING_KD)

      drivekP.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KP)
      drivekI.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KI)
      drivekD.initDefault(DrivetrainConstants.PID.SIM_DRIVE_KD)
    }

    driveKV.initDefault(DrivetrainConstants.PID.DRIVE_KV)
    driveKA.initDefault(DrivetrainConstants.PID.DRIVE_KA)
  }

  fun updateInputs() {
    io.updateInputs(inputs)
  }

  fun periodic() {
    positionDeltas.add(
      SwerveModulePosition(
        (inputs.drivePosition - lastDrivePosition).inMeters,
        inputs.steeringPosition.inRotation2ds
      )
    )
    lastDrivePosition = inputs.drivePosition

    //    val deltaCount =
    //      Math.min(inputs.odometryDrivePositions.size, inputs.odometrySteeringPositions.size)
    //
    //    for (i in 0 until deltaCount) {
    //      val newDrivePosition = inputs.odometryDrivePositions[i]
    //      val newSteeringAngle = inputs.odometrySteeringPositions[i]
    //      positionDeltas.add(
    //        SwerveModulePosition(
    //          (newDrivePosition - lastDrivePosition).inMeters, newSteeringAngle.inRotation2ds
    //        )
    //      )
    //      lastDrivePosition = newDrivePosition
    //    }
    //
    //    if (positionDeltas.size > 0) {
    //      Logger.recordOutput("Drivetrain/PositionDeltas", positionDeltas[0].distanceMeters)
    //    } else {
    //      Logger.recordOutput("Drivetrain/PositionDeltas", -1337)
    //  }

    // Updating SwerveModulePosition every loop cycle
    modulePosition.distanceMeters = inputs.drivePosition.inMeters
    modulePosition.angle = inputs.steeringPosition.inRotation2ds

    if (steeringkP.hasChanged() || steeringkI.hasChanged() || steeringkD.hasChanged()) {
      io.configureSteeringPID(steeringkP.get(), steeringkI.get(), steeringkD.get())
    }

    if (steeringMaxVel.hasChanged() || steeringMaxAccel.hasChanged()) {
      io.configureSteeringMotionMagic(steeringMaxVel.get(), steeringMaxAccel.get())
    }

    if (drivekP.hasChanged() ||
      drivekI.hasChanged() ||
      drivekD.hasChanged() ||
      driveKV.hasChanged() ||
      driveKA.hasChanged()
    ) {
      io.configureDrivePID(
        drivekP.get(), drivekI.get(), drivekD.get(), driveKV.get(), driveKA.get()
      )
    }

    Logger.processInputs(io.label, inputs)
    Logger.recordOutput(
      "${io.label}/driveSpeedSetpointMetersPerSecond",
      if (!shouldInvert) speedSetPoint.inMetersPerSecond else -speedSetPoint.inMetersPerSecond
    )
    Logger.recordOutput(
      "${io.label}/driveAccelSetpointMetersPerSecondSq",
      accelerationSetPoint.inMetersPerSecondPerSecond
    )
    Logger.recordOutput("${io.label}/steeringSetpointRadians", steeringSetPoint.inRadians)
    Logger.recordOutput(
      "${io.label}/steeringValueDegreesWithMod", inputs.steeringPosition.inDegrees.IEEErem(360.0)
    )

    Logger.recordOutput("SwerveModule/SpeedSetPoint", speedSetPoint.inMetersPerSecond)
    Logger.recordOutput("SwerveModule/SteeringSetPoint", steeringSetPoint.inRadians)
    Logger.recordOutput(
      "SwerveModule/AccelerationSetPoint", accelerationSetPoint.inMetersPerSecondPerSecond
    )
    Logger.recordOutput(
      "SwerveModule/SteeringError", (steeringSetPoint - inputs.steeringPosition).inRadians
    )
  }

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using feed forward.
   *
   * @param steering The angular position desired for the swerve module to be set to
   * @param speed The speed desired for the swerve module to reach
   * @param acceleration The linear acceleration used to calculate how to reach the desired speed
   */
  fun set(
    steering: Angle,
    speed: LinearVelocity,
    acceleration: LinearAcceleration = 0.0.meters.perSecond.perSecond,
    optimize: Boolean = true
  ) {
    if (speed == 0.feet.perSecond) {
      io.setOpenLoop(steeringSetPoint, 0.0.meters.perSecond)
      return
    }
    var steeringDifference =
      (steering - inputs.steeringPosition).inRadians.IEEErem(2 * Math.PI).radians

    shouldInvert = (steeringDifference.absoluteValue > (Math.PI / 2).radians) && optimize

    if (shouldInvert) {
      steeringDifference -= Math.PI.withSign(steeringDifference.inRadians).radians
    }

    speedSetPoint =
      if (shouldInvert) {
        speed * -1
      } else {
        speed
      }
    accelerationSetPoint =
      if (shouldInvert) {
        acceleration * -1
      } else {
        acceleration
      }
    steeringSetPoint = inputs.steeringPosition + steeringDifference

    //    io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
    io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
  }

  fun setOpenLoop(steering: Angle, speed: LinearVelocity, optimize: Boolean = true) {
    var steeringDifference =
      (steering - inputs.steeringPosition).inRadians.IEEErem(2 * Math.PI).radians
    shouldInvert = steeringDifference.absoluteValue > (Math.PI / 2).radians && optimize
    if (shouldInvert) {
      steeringDifference -= Math.PI.withSign(steeringDifference.inRadians).radians
    }
    val outputSpeed =
      if (shouldInvert) {
        speed * -1
      } else {
        speed
      }
    steeringSetPoint = inputs.steeringPosition + steeringDifference
    io.setOpenLoop(steeringSetPoint, outputSpeed)
  }

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using open loop control.
   *
   * @param desiredState The desired SwerveModuleState. Contains desired angle as well as X and Y
   * velocities
   */
  fun setPositionOpenLoop(desiredState: SwerveModuleState, optimize: Boolean = true) {
    if (optimize) {
      Logger.recordOutput("${io.label}/desiredAngleRadians", desiredState.angle.radians)
      //      val adjustedState: SwerveModuleState
      //      if ((inputs.steeringPosition +
      // 360.degrees).minus(desiredState.angle.degrees.degrees).absoluteValue <=
      // (inputs.steeringPosition).minus(desiredState.angle.degrees.degrees).absoluteValue){
      //        adjustedState = SwerveModuleState(desiredState.speedMetersPerSecond,
      // desiredState.angle)
      //      } else {
      //        adjustedState = desiredState
      //      }
      Logger.recordOutput(
        "${io.label}/minimizedDeltaRadians", (inputs.steeringPosition + 360.degrees).inRadians
      )

      val optimizedState =
        SwerveModuleState.optimize(desiredState, inputs.steeringPosition.inRotation2ds)
      io.setOpenLoop(
        optimizedState.angle.angle,
        optimizedState.speedMetersPerSecond.meters.perSecond *
          Math.cos(
            abs(
              (optimizedState.angle.angle - inputs.steeringPosition)
                .inRadians
            )
          ) // consider desaturating wheel speeds here if it doesn't work
        // from drivetrain
      )
      Logger.recordOutput("${io.label}/steeringSetpointOptimized", optimizedState.angle.degrees)
    } else {
      io.setOpenLoop(
        desiredState.angle.angle,
        desiredState.speedMetersPerSecond.meters.perSecond *
          Math.cos(abs((desiredState.angle.angle - inputs.steeringPosition).inRadians))
      )
      Logger.recordOutput("${io.label}/steeringSetpointNonOptimized", desiredState.angle.degrees)
    }
  }

  /**
   * Sets the swerve module to the specified angular and X & Y velocities using feed forward.
   *
   * @param desiredVelState The desired SwerveModuleState. Contains desired angle as well as X and Y
   * velocities
   * @param desiredAccelState The desired SwerveModuleState that contains desired acceleration
   * vectors.
   * @param optimize Whether velocity and acceleration vectors should be optimized (if possible)
   */
  fun setPositionClosedLoop(
    desiredVelState: SwerveModuleState,
    desiredAccelState: SwerveModuleState,
    optimize: Boolean = true
  ) {
    if (optimize) {
      val optimizedVelState =
        SwerveModuleState.optimize(desiredVelState, inputs.steeringPosition.inRotation2ds)
      val optimizedAccelState =
        SwerveModuleState.optimize(desiredAccelState, inputs.steeringPosition.inRotation2ds)

      steeringSetPoint = optimizedVelState.angle.angle
      speedSetPoint = optimizedVelState.speedMetersPerSecond.meters.perSecond
      accelerationSetPoint = optimizedAccelState.speedMetersPerSecond.meters.perSecond.perSecond

      io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
    } else {
      steeringSetPoint = desiredVelState.angle.angle
      speedSetPoint = desiredVelState.speedMetersPerSecond.meters.perSecond
      accelerationSetPoint = desiredAccelState.speedMetersPerSecond.meters.perSecond.perSecond

      io.setClosedLoop(steeringSetPoint, speedSetPoint, accelerationSetPoint)
    }
  }

  /** Creates event of the current potentiometer value as needs to be manually readjusted. */
  fun resetModuleZero() {
    io.resetModuleZero()
  }

  /** Zeros the steering motor */
  fun zeroSteering(isInAutonomous: Boolean = false) {
    io.zeroSteering(isInAutonomous)
  }

  /** Zeros the drive motor */
  fun zeroDrive() {
    io.zeroDrive()
  }

  fun setDriveBrakeMode(brake: Boolean) {
    io.setDriveBrakeMode(brake)
  }

  fun setSteeringBrakeMode(brake: Boolean) {
    io.setSteeringBrakeMode(brake)
  }

  fun runCharacterization(input: ElectricalPotential) {
    io.runCharacterization(input)
  }
}
