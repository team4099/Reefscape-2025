package com.team4099.robot2025.subsystems.drivetrain.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.GyroConstants
import com.team4099.utils.threads.PhoenixOdometryThread
import com.team4099.utils.threads.SparkMaxOdometryThread
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import java.util.Queue
import kotlin.math.IEEErem

object GyroIOPigeon2 : GyroIO {
  private var pigeon2 = Pigeon2(Constants.Gyro.PIGEON_2_ID, Constants.Universal.CANIVORE_NAME)

  private val yawSignal = pigeon2.yaw
  private val upTimeSignal = pigeon2.upTime
  private val pitchSignal = pigeon2.pitch
  private val rollSignal = pigeon2.roll
  private val rollVelSignal = pigeon2.angularVelocityXWorld
  private val pitchVelSignal = pigeon2.angularVelocityYWorld
  private val yawVelSignal = pigeon2.angularVelocityZWorld

  private val isConnected: Boolean
    get() {
      return upTimeSignal.valueAsDouble > 0.0
    }

  var gyroYawOffset: Angle = 0.0.degrees
  var gyroPitchOffset: Angle = 0.0.degrees
  var gyroRollOffset: Angle = 0.0.degrees

  val yawPositionQueue: Queue<Double>

  val rawGyro: Angle = 0.0.degrees

  /** The current angle of the drivetrain. */
  val gyroYaw: Angle
    get() {
      if (isConnected) {
        var rawYaw = yawSignal.valueAsDouble + gyroYawOffset.inDegrees
        rawYaw += DrivetrainConstants.GYRO_RATE_COEFFICIENT * gyroYawRate.inDegreesPerSecond
        return rawYaw.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroPitch: Angle
    get() {
      if (isConnected) {
        val rawPitch = pitchSignal.valueAsDouble + gyroPitchOffset.inDegrees
        return rawPitch.IEEErem(360.0).degrees
      } else {
        return (-1.337).degrees
      }
    }

  val gyroRoll: Angle
    get() {
      if (isConnected) {
        val rawRoll = rollSignal.valueAsDouble + gyroRollOffset.inDegrees
        return rawRoll.IEEErem(360.0).degrees
      } else {
        return -1.337.degrees
      }
    }

  val gyroYawRate: AngularVelocity
    get() {
      if (isConnected) {
        return yawVelSignal.valueAsDouble.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroPitchRate: AngularVelocity
    get() {
      if (isConnected) {
        return pitchVelSignal.valueAsDouble.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  val gyroRollRate: AngularVelocity
    get() {
      if (isConnected) {
        return rollVelSignal.valueAsDouble.degrees.perSecond
      } else {
        return -1.337.degrees.perSecond
      }
    }

  init {
    val pigeon2Configuration = Pigeon2Configuration()
    pigeon2Configuration.MountPose.MountPosePitch = GyroConstants.mountPitch.inRadians
    pigeon2Configuration.MountPose.MountPoseYaw = GyroConstants.mountYaw.inRadians
    pigeon2Configuration.MountPose.MountPoseRoll = GyroConstants.mountRoll.inRadians

    yawPositionQueue =
      if (Constants.Drivetrain.DRIVETRAIN_TYPE ==
        Constants.Drivetrain.DrivetrainType.PHOENIX_TALON
      ) {
        PhoenixOdometryThread.getInstance().registerSignal(pigeon2, pigeon2.yaw)
      } else {
        SparkMaxOdometryThread.getInstance().registerSignal { pigeon2.yaw.valueAsDouble }
      }

    // TODO look into more pigeon configuration stuff
    pigeon2.configurator.apply(pigeon2Configuration)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    BaseStatusSignal.refreshAll(
      yawSignal,
      upTimeSignal,
      pitchSignal,
      rollSignal,
      yawVelSignal,
      pitchVelSignal,
      rollVelSignal
    )
    inputs.rawGyroYaw = yawSignal.valueAsDouble.degrees

    inputs.gyroConnected = isConnected

    inputs.gyroYaw = gyroYaw
    inputs.gyroPitch = gyroPitch
    inputs.gyroRoll = gyroRoll

    inputs.gyroYawRate = gyroYawRate
    inputs.gyroPitchRate = gyroPitchRate
    inputs.gyroRollRate = gyroRollRate

    inputs.odometryYawPositions =
      (yawPositionQueue.stream().map { value: Double -> value.degrees }.toList() as List<Angle>)
        .toMutableList()
    yawPositionQueue.clear()

    Logger.recordOutput("Gyro/rawYawDegrees", pigeon2.yaw.value)
  }

  override fun zeroGyroYaw(toAngle: Angle) {
    gyroYawOffset = toAngle - pigeon2.yaw.valueAsDouble.IEEErem(360.0).degrees
  }

  override fun zeroGyroPitch(toAngle: Angle) {
    gyroPitchOffset = toAngle - pigeon2.pitch.valueAsDouble.IEEErem(360.0).degrees
  }

  override fun zeroGyroRoll(toAngle: Angle) {
    gyroRollOffset = toAngle - pigeon2.roll.valueAsDouble.IEEErem(360.0).degrees
  }
}
