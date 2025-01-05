package com.team4099.robot2023.subsystems.drivetrain.drive

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.Alert
import com.team4099.robot2023.util.FMSData
import com.team4099.robot2023.util.FieldFrameEstimator
import com.team4099.robot2023.util.Velocity2d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class Drivetrain(val gyroIO: GyroIO, swerveModuleIOs: DrivetrainIO) : SubsystemBase() {
  private val gyroNotConnectedAlert =
    Alert(
      "Gyro is not connected, field relative driving will be significantly worse.",
      Alert.AlertType.ERROR
    )
  val swerveModules = swerveModuleIOs.getSwerveModules() // FL, FR, BL, BR

  val gyroInputs = GyroIO.GyroIOInputs()

  private var gyroYawOffset = 0.0.radians

  // TODO: tune these
  private var fieldFrameEstimator = FieldFrameEstimator(VecBuilder.fill(0.003, 0.003, 0.0001))

  private var angularVelocityTarget = 0.degrees.perSecond

  private var targetedDriveVector = Pair(0.meters.perSecond, 0.meters.perSecond)

  private var isFieldOriented = true

  var targetedChassisSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    private set

  var targetedChassisAccels = edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    private set

  var isInAutonomous = false
    private set

  var targetPose: Pose2d = Pose2d(0.0.meters, 0.0.meters, 0.0.radians)

  private var drift: Transform2d = Transform2d(Translation2d(), 0.0.radians)

  var fieldVelocity = Velocity2d(0.0.meters.perSecond, 0.0.meters.perSecond)
    private set

  var robotVelocity = Velocity2d(0.0.meters.perSecond, 0.0.meters.perSecond)

  private var omegaVelocity = 0.0.radians.perSecond

  private var characterizationInput = 0.0.volts

  var lastGyroYaw = { gyroInputs.gyroYaw }

  var currentState: DrivetrainState = DrivetrainState.UNINITIALIZED
    private set

  private val testAngle =
    LoggedTunableValue("Drivetrain/testAngle", 0.degrees, Pair({ it.inDegrees }, { it.degrees }))

  private val swerveModuleID =
    LoggedTunableValue("Drivetrain/testModule", 0.degrees, Pair({ it.inDegrees }, { it.degrees }))

  val closestAlignmentAngle: Angle
    get() {
      for (angle in -180..90 step 90) {
        if ((odomTRobot.rotation - angle.degrees).absoluteValue <= 45.degrees) {
          return angle.degrees
        }
      }
      return 0.0.degrees
    }

  var currentRequest: DrivetrainRequest = DrivetrainRequest.ZeroSensors()
    set(value) {
      when (value) {
        is DrivetrainRequest.OpenLoop -> {
          angularVelocityTarget = value.angularVelocity
          targetedDriveVector = value.driveVector
          isFieldOriented = value.fieldOriented
        }
        is DrivetrainRequest.ClosedLoop -> {
          targetedChassisSpeeds = value.chassisSpeeds
          targetedChassisAccels = value.chassisAccels
        }
        is DrivetrainRequest.ZeroSensors -> {
          isInAutonomous = value.isInAutonomous
        }
        is DrivetrainRequest.Characterize -> {
          characterizationInput = value.voltage
        }
        else -> {}
      }
      field = value
    }

  private val frontLeftWheelLocation =
    Translation2d(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val frontRightWheelLocation =
    Translation2d(
      DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backLeftWheelLocation =
    Translation2d(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )
  private val backRightWheelLocation =
    Translation2d(
      -DrivetrainConstants.DRIVETRAIN_LENGTH / 2, -DrivetrainConstants.DRIVETRAIN_WIDTH / 2
    )

  val moduleTranslations =
    listOf(
      frontLeftWheelLocation,
      frontRightWheelLocation,
      backLeftWheelLocation,
      backRightWheelLocation
    )

  private val swerveDriveKinematics =
    SwerveDriveKinematics(
      frontLeftWheelLocation.translation2d,
      frontRightWheelLocation.translation2d,
      backLeftWheelLocation.translation2d,
      backRightWheelLocation.translation2d
    )

  private var swerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray()
    )

  private var undriftedSwerveDriveOdometry =
    SwerveDriveOdometry(
      swerveDriveKinematics,
      gyroInputs.gyroYaw.inRotation2ds,
      swerveModules.map { it.modulePosition }.toTypedArray()
    )

  private var setPointStates =
    mutableListOf(
      SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState()
    )

  val odomTRobot: Pose2d
    get() = Pose2d(swerveDriveOdometry.poseMeters)

  // NOTE(parth): This should be expected to be noisy. Avoid using this directly if possible.
  val fieldTRobot: Pose2d
    get() =
      (fieldFrameEstimator.getLatestOdometryTField().inverse() + odomTRobot.asTransform2d())
        .asPose2d()

  val odomTField: Transform2d
    get() = fieldFrameEstimator.getLatestOdometryTField()

  val odomTSpeaker: Transform2d
    get() = fieldFrameEstimator.getLatestOdometryTSpeaker()

  private var undriftedPose: Pose2d
    get() = Pose2d(undriftedSwerveDriveOdometry.poseMeters)
    set(value) {
      undriftedSwerveDriveOdometry.resetPosition(
        gyroInputs.gyroYaw.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        value.pose2d
      )
    }

  private var rawGyroAngle = odomTRobot.rotation

  private val lastModulePositions =
    arrayOf(
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition()
    )

  init {
    // Wheel speeds
    zeroSteering()
  }

  override fun periodic() {
    val startTime = Clock.realTimestamp
    gyroIO.updateInputs(gyroInputs)

    // Read new gyro and wheel data from sensors
    //    odometryLock.lock() // Prevents odometry updates while reading data

    swerveModules.forEach { it.updateInputs() }
    //    odometryLock.unlock()

    Logger.processInputs("Gyro", gyroInputs)

    gyroNotConnectedAlert.set(!gyroInputs.gyroConnected)

    swerveModules.forEach { it.periodic() }

    // Update field velocity
    val measuredStates = arrayOfNulls<SwerveModuleState>(4)
    for (i in 0..3) {
      measuredStates[i] =
        SwerveModuleState(
          swerveModules[i].inputs.driveVelocity.inMetersPerSecond,
          swerveModules[i].inputs.steeringPosition.inRotation2ds
        )
    }
    val chassisState: ChassisSpeeds =
      ChassisSpeeds(swerveDriveKinematics.toChassisSpeeds(measuredStates))
    val fieldVelCalculated =
      Translation2d(
        chassisState.vx.inMetersPerSecond.meters, chassisState.vy.inMetersPerSecond.meters
      )
        .rotateBy(odomTRobot.rotation) // we don't use this but it's there if you want it ig

    robotVelocity = Velocity2d(chassisState.vx, chassisState.vy)
    fieldVelocity = Velocity2d(fieldVelCalculated.x.perSecond, fieldVelCalculated.y.perSecond)

    omegaVelocity = chassisState.omega
    if (!gyroInputs.gyroConnected) {
      gyroInputs.gyroYawRate = omegaVelocity
      rawGyroAngle += Constants.Universal.LOOP_PERIOD_TIME * gyroInputs.gyroYawRate
      gyroInputs.gyroYaw = rawGyroAngle + gyroYawOffset
    }

    // updating odometry every loop cycle
    updateOdometry()

    Logger.recordOutput(
      "FieldFrameEstimator/odomTSpeaker",
      fieldFrameEstimator.getLatestOdometryTSpeaker().transform2d
    )

    Logger.recordOutput("Drivetrain/OdometryGyroRotationValue", odomTRobot.rotation.inDegrees)

    Logger.recordOutput(
      "Drivetrain/xRobotVelocityMetersPerSecond", robotVelocity.x.inMetersPerSecond
    )
    Logger.recordOutput(
      "Drivetrain/xRobotVelocityMetersPerSecond", robotVelocity.x.inMetersPerSecond
    )
    Logger.recordOutput(
      "Drivetrain/xFieldVelocityMetersPerSecond", fieldVelocity.x.inMetersPerSecond
    )
    Logger.recordOutput(
      "Drivetrain/yFieldVelocityMetersPerSecond", fieldVelocity.y.inMetersPerSecond
    )

    Logger.processInputs("Drivetrain/Gyro", gyroInputs)
    Logger.recordOutput(
      VisionConstants.POSE_TOPIC_NAME,
      doubleArrayOf(odomTRobot.x.inMeters, odomTRobot.y.inMeters, odomTRobot.rotation.inRadians)
    )
    Logger.recordOutput("" + "FieldRelativePose/robotPose", fieldTRobot.pose2d)

    Logger.recordOutput("Drivetrain/ModuleStates", *measuredStates)
    Logger.recordOutput("Drivetrain/setPointStates", *setPointStates.toTypedArray())

    Logger.recordOutput(
      VisionConstants.POSE_TOPIC_NAME,
      doubleArrayOf(odomTRobot.x.inMeters, odomTRobot.y.inMeters, odomTRobot.rotation.inRadians)
    )
    Logger.recordOutput(
      "Odometry/pose3d",
      Pose3d(
        odomTRobot.x,
        odomTRobot.y,
        1.0.meters,
        Rotation3d(gyroInputs.gyroRoll, gyroInputs.gyroPitch, gyroInputs.gyroYaw)
      )
        .pose3d
    )

    Logger.recordOutput("FieldFrameEstimator/odomTField", odomTField.transform2d)

    Logger.recordOutput(
      "Odometry/targetPose",
      doubleArrayOf(targetPose.x.inMeters, targetPose.y.inMeters, targetPose.rotation.inRadians)
    )

    Logger.recordOutput(
      "LoggedRobot/Subsystems/DrivetrainLoopTimeMS",
      (Clock.realTimestamp - startTime).inMilliseconds
    )

    // Log the current state
    Logger.recordOutput("Drivetrain/currentState", currentState.toString())
    // Log the current state
    Logger.recordOutput("Drivetrain/currentRequest", currentRequest.javaClass.toString())

    var nextState = currentState

    when (currentState) {
      DrivetrainState.UNINITIALIZED -> {
        // Transitions
        nextState = DrivetrainState.ZEROING_SENSORS
      }
      DrivetrainState.ZEROING_SENSORS -> {
        zeroSensors(isInAutonomous)
        // Transitions
        currentRequest = DrivetrainRequest.Idle()
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.IDLE -> {
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.OPEN_LOOP -> {
        // Outputs
        setOpenLoop(angularVelocityTarget, targetedDriveVector, isFieldOriented)
        Logger.recordOutput(
          "Drivetrain/TargetVelocityX", targetedDriveVector.first.inMetersPerSecond
        )
        Logger.recordOutput(
          "Drivetrain/TargetVelocityY", targetedDriveVector.second.inMetersPerSecond
        )
        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.CLOSED_LOOP -> {
        // Outputs
        setClosedLoop(targetedChassisSpeeds, targetedChassisAccels)

        Logger.recordOutput("Drivetrain/TargetChassisSpeeds", targetedChassisSpeeds)
        Logger.recordOutput("Drivetrain/TargetChassisAccels", targetedChassisAccels)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.CHARACTERIZE -> {
        swerveModules.forEach { it.runCharacterization(characterizationInput) }

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
      DrivetrainState.LOCK_WHEELS -> {
        lockWheels()
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  private fun updateOdometry() {
    for (i in 0 until 4) {
      lastModulePositions[i] =
        SwerveModulePosition(
          swerveModules[i].inputs.drivePosition.inMeters,
          swerveModules[i].inputs.steeringPosition.inRotation2ds
        )
    }

    // TODO(parth): The thing I removed here may have removed support for driving field oriented w/o
    // a gyro. Check before merging
    // TODO(parth): For this to work our gyro rate coefficient thing needs to be correct
    swerveDriveOdometry.update(gyroInputs.gyroYaw.inRotation2ds, lastModulePositions)
    fieldFrameEstimator.addDriveData(Clock.fpgaTime, odomTRobot)

    // reversing the drift to store the sim ground truth pose
    if (RobotBase.isSimulation() && Constants.Tuning.SIMULATE_DRIFT) {
      val undriftedModules = arrayOfNulls<SwerveModulePosition>(4)
      for (i in 0 until 4) {
        undriftedModules[i] =
          SwerveModulePosition(
            (
              swerveModules[i].modulePosition.distanceMeters.meters -
                swerveModules[i].inputs.drift
              )
              .inMeters,
            swerveModules[i].modulePosition.angle
          )
      }
      undriftedSwerveDriveOdometry.update((gyroInputs.gyroYaw).inRotation2ds, undriftedModules)

      drift = undriftedPose.minus(odomTRobot)
      Logger.recordOutput(VisionConstants.SIM_POSE_TOPIC_NAME, undriftedPose.pose2d)
    }
  }

  /**
   * @param angularVelocity Represents the angular velocity of the chassis
   * @param driveVector Pair of linear velocities: First is X vel, second is Y vel
   * @param fieldOriented Are the chassis speeds driving relative to field (aka use gyro or not)
   */
  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    fieldOriented: Boolean = true
  ) {

    Logger.recordOutput("Drivetrain/isFieldOriented", fieldOriented)
    // flip the direction base don alliance color
    val flipDrive = if (FMSData.allianceColor == DriverStation.Alliance.Red) -1 else 1
    val allianceFlippedDriveVector =
      Pair(driveVector.first * flipDrive, driveVector.second * flipDrive)

    Logger.recordOutput(
      "Drivetrain/driveVectorFirst", allianceFlippedDriveVector.first.inMetersPerSecond
    )
    Logger.recordOutput(
      "Drivetrain/driveVectorSecond", allianceFlippedDriveVector.second.inMetersPerSecond
    )

    val swerveModuleStates: Array<SwerveModuleState>
    var desiredChassisSpeeds: ChassisSpeeds

    // calculated chasis speeds, apply field oriented transformation
    if (fieldOriented) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
          angularVelocity,
          odomTRobot.rotation
        )
    } else {
      desiredChassisSpeeds =
        ChassisSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
          angularVelocity,
        )
    }

    Logger.recordOutput(
      "Drivetrain/omegaDegreesPerSecond", desiredChassisSpeeds.omega.inDegreesPerSecond
    )

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      val velocityTransform =
        Transform2d(
          Translation2d(
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vx,
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vy
          ),
          Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.omega
        )

      val twistToNextPose: Twist2d = velocityTransform.log()

      desiredChassisSpeeds =
        ChassisSpeeds(
          (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
        )
    }

    // convert target chassis speeds to individual module setpoint states
    swerveModuleStates =
      swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds.chassisSpeedsWPILIB)
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
    )
    setPointStates = swerveModuleStates.toMutableList()

    // set each module openloop based on corresponding states
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionOpenLoop(swerveModuleStates[moduleIndex])
    }
  }

  fun setOpenLoop(
    angularVelocity: AngularVelocity,
    driveVector: Pair<LinearVelocity, LinearVelocity>,
    chassisAccels: edu.wpi.first.math.kinematics.ChassisSpeeds =
      edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0),
    fieldOriented: Boolean = true
  ) {

    Logger.recordOutput("Drivetrain/isFieldOriented", fieldOriented)
    // flip the direction base don alliance color
    val flipDrive = if (FMSData.allianceColor == DriverStation.Alliance.Red) -1 else 1
    val allianceFlippedDriveVector =
      Pair(driveVector.first * flipDrive, driveVector.second * flipDrive)

    Logger.recordOutput(
      "Drivetrain/driveVectorFirst", allianceFlippedDriveVector.first.inMetersPerSecond
    )
    Logger.recordOutput(
      "Drivetrain/driveVectorSecond", allianceFlippedDriveVector.second.inMetersPerSecond
    )

    val swerveModuleStates: Array<SwerveModuleState>
    var desiredChassisSpeeds: ChassisSpeeds

    // calculated chasis speeds, apply field oriented transformation
    if (fieldOriented) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
          angularVelocity,
          odomTRobot.rotation
        )
    } else {
      desiredChassisSpeeds =
        ChassisSpeeds(
          allianceFlippedDriveVector.first,
          allianceFlippedDriveVector.second,
          angularVelocity,
        )
    }

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      val velocityTransform =
        Transform2d(
          Translation2d(
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vx,
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vy
          ),
          Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.omega
        )

      val twistToNextPose: Twist2d = velocityTransform.log()

      desiredChassisSpeeds =
        ChassisSpeeds(
          (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
        )
    }

    // convert target chassis speeds to individual module setpoint states
    swerveModuleStates =
      swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds.chassisSpeedsWPILIB)
    val accelSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisAccels)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DrivetrainConstants.DRIVE_SETPOINT_MAX.inMetersPerSecond
    )
    setPointStates = swerveModuleStates.toMutableList()

    // set each module openloop based on corresponding states
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionClosedLoop(
        swerveModuleStates[moduleIndex], accelSwerveModuleStates[moduleIndex]
      )
    }
  }

  /**
   * Sets the drivetrain to the specified angular and X & Y velocities based on the current angular
   * and linear acceleration. Calculates both angular and linear velocities and acceleration and
   * calls setPositionClosedLoop for each SwerveModule object.
   *
   * @param angularVelocity The angular velocity of a specified drive
   * @param driveVector.first The linear velocity on the X axis
   * @param driveVector.second The linear velocity on the Y axis
   * @param angularAcceleration The angular acceleration of a specified drive
   * @param driveAcceleration.first The linear acceleration on the X axis
   * @param driveAcceleration.second The linear acceleration on the Y axis
   * @param fieldOriented Defines whether module states are calculated relative to field
   */
  fun setClosedLoop(
    chassisSpeeds: edu.wpi.first.math.kinematics.ChassisSpeeds,
    chassisAccels: edu.wpi.first.math.kinematics.ChassisSpeeds =
      edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
  ) {
    var chassisSpeeds = chassisSpeeds

    if (DrivetrainConstants.MINIMIZE_SKEW) {
      val velocityTransform =
        Transform2d(
          Translation2d(
            Constants.Universal.LOOP_PERIOD_TIME *
              chassisSpeeds.vxMetersPerSecond.meters.perSecond,
            Constants.Universal.LOOP_PERIOD_TIME *
              chassisSpeeds.vyMetersPerSecond.meters.perSecond
          ),
          Constants.Universal.LOOP_PERIOD_TIME *
            chassisSpeeds.omegaRadiansPerSecond.radians.perSecond
        )

      val twistToNextPose: Twist2d = velocityTransform.log()

      chassisSpeeds =
        ChassisSpeeds(
          (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
          (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
        )
          .chassisSpeedsWPILIB
    }

    val velSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    val accelSwerveModuleStates: Array<SwerveModuleState> =
      swerveDriveKinematics.toSwerveModuleStates(chassisAccels)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      velSwerveModuleStates, DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond
    )

    setPointStates = velSwerveModuleStates.toMutableList()

    // Once we have all of our states obtained for both velocity and acceleration, apply these
    // states to each swerve module
    for (moduleIndex in 0 until DrivetrainConstants.WHEEL_COUNT) {
      swerveModules[moduleIndex].setPositionClosedLoop(
        velSwerveModuleStates[moduleIndex], accelSwerveModuleStates[moduleIndex]
      )
    }
  }

  fun resetModuleZero() {
    swerveModules.forEach { it.resetModuleZero() }
  }

  /** Zeros all the sensors on the drivetrain. */
  fun zeroSensors(isInAutonomous: Boolean) {
    zeroGyroPitch()
    zeroGyroRoll()
    zeroSteering(isInAutonomous)

    if (!isInAutonomous) {
      zeroDrive()
    }
  }

  /** Resets the field frame estimator given some current pose of the robot. */
  fun resetFieldFrameEstimator(fieldTRobot: Pose2d) {
    fieldFrameEstimator.resetFieldFrameFilter(
      odomTRobot.asTransform2d() + fieldTRobot.asTransform2d().inverse()
    )
  }

  fun tempZeroGyroYaw(toAngle: Angle = 0.0.degrees) {
    swerveDriveOdometry.resetPosition(
      gyroInputs.gyroYaw.inRotation2ds,
      lastModulePositions,
      Pose2d(odomTRobot.x, odomTRobot.y, toAngle).pose2d
    )
  }

  /**
   * Sets the gyroOffset in such a way that when added to the gyro angle it gives back toAngle.
   *
   * @param toAngle Zeros the gyro to the value
   */
  fun zeroGyroYaw(toAngle: Angle = 0.degrees) {
    // TODO(parth): This feels incorrect -- I think the first arg should be the gyro angle and the
    // undrifted pose should be updated to toAngle
    if (RobotBase.isSimulation()) {
      // NOTE(parth): The gyro itself should never need to be reset in-match on a real robot, the
      // odometry can be updated directly
      gyroIO.zeroGyroYaw(toAngle)
      undriftedSwerveDriveOdometry.resetPosition(
        toAngle.inRotation2ds,
        swerveModules.map { it.modulePosition }.toTypedArray(),
        undriftedPose.pose2d
      )
    }

    // TODO(parth): Update the field frame estimator's transform here too, otherwise it will need to
    // re-converge
    swerveDriveOdometry.resetPosition(
      gyroInputs.gyroYaw.inRotation2ds,
      lastModulePositions,
      Pose2d(odomTRobot.x, odomTRobot.y, toAngle).pose2d
    )

    fieldFrameEstimator.resetFieldFrameFilter(
      Transform2d(odomTField.translation, gyroInputs.gyroYaw)
    )

    if (!(gyroInputs.gyroConnected)) {
      gyroYawOffset = toAngle - rawGyroAngle
    }
  }

  fun zeroGyroPitch(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroPitch(toAngle)
  }

  fun zeroGyroRoll(toAngle: Angle = 0.0.degrees) {
    gyroIO.zeroGyroRoll(toAngle)
  }

  /** Zeros the steering motors for each swerve module. */
  fun zeroSteering(isInAutonomous: Boolean = false) {
    swerveModules.forEach { it.zeroSteering(isInAutonomous) }
  }

  /** Zeros the drive motors for each swerve module. */
  private fun zeroDrive() {
    swerveModules.forEach { it.zeroDrive() }
  }

  fun driveSetpointTestCommand(): Command {
    return runOnce {
      swerveModules[swerveModuleID.get().inDegrees.toInt()].setOpenLoop(
        testAngle.get(), 0.meters.perSecond, false
      )
    }
  }

  fun addVisionData(visionData: List<TimestampedVisionUpdate>) {
    fieldFrameEstimator.addVisionData(visionData)
  }

  fun addSpeakerVisionData(visionData: TimestampedTrigVisionUpdate) {
    fieldFrameEstimator.addSpeakerVisionData(visionData)
  }

  fun lockWheels() {
    swerveModules[0].setOpenLoop(DrivetrainConstants.FL_LOCKING_ANGLE, 0.meters.perSecond, true)
    swerveModules[1].setOpenLoop(DrivetrainConstants.FR_LOCKING_ANGLE, 0.meters.perSecond, true)
    swerveModules[2].setOpenLoop(DrivetrainConstants.BL_LOCKING_ANGLE, 0.meters.perSecond, true)
    swerveModules[3].setOpenLoop(DrivetrainConstants.BR_LOCKING_ANGLE, 0.meters.perSecond, true)
  }

  fun lockWheelsCommand(): Command {
    return Commands.runOnce({ currentRequest = Request.DrivetrainRequest.LockWheels() })
  }

  companion object {
    // Drivetrain multithreading
    var odometryLock: Lock = ReentrantLock()
    fun setOdometryLock(Locked: Boolean) {
      if (Locked) {
        odometryLock.lock()
      } else {
        odometryLock.unlock()
      }
    }

    // Drivetrain states for state machine.
    enum class DrivetrainState {
      UNINITIALIZED,
      IDLE,
      ZEROING_SENSORS,
      OPEN_LOOP,
      LOCK_WHEELS,
      CLOSED_LOOP,
      CHARACTERIZE;

      inline fun equivalentToRequest(request: Request.DrivetrainRequest): Boolean {
        return (
          (request is DrivetrainRequest.ZeroSensors && this == ZEROING_SENSORS) ||
            (request is DrivetrainRequest.OpenLoop && this == OPEN_LOOP) ||
            (request is DrivetrainRequest.ClosedLoop && this == CLOSED_LOOP) ||
            (request is DrivetrainRequest.Idle && this == IDLE) ||
            (request is DrivetrainRequest.LockWheels && this == LOCK_WHEELS) ||
            (request is DrivetrainRequest.Characterize && this == CHARACTERIZE)
          )
      }
    }

    inline fun fromRequestToState(request: Request.DrivetrainRequest): DrivetrainState {
      return when (request) {
        is DrivetrainRequest.OpenLoop -> DrivetrainState.OPEN_LOOP
        is DrivetrainRequest.ClosedLoop -> DrivetrainState.CLOSED_LOOP
        is DrivetrainRequest.ZeroSensors -> DrivetrainState.ZEROING_SENSORS
        is DrivetrainRequest.Idle -> DrivetrainState.IDLE
        is DrivetrainRequest.LockWheels -> DrivetrainState.LOCK_WHEELS
        is DrivetrainRequest.Characterize -> DrivetrainState.CHARACTERIZE
      }
    }
  }
}
