package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.logging.toDoubleArray
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.util.FMSData
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonUtils
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import java.util.function.Consumer
import java.util.function.Supplier

class Vision(vararg cameras: CameraIO) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  var drivetrainOdometry: () -> Pose2d = { Pose2d() }
  var robotTSpeaker: Translation3d = Translation3d()
  var trustedRobotDistanceToTarget: Length = 0.meters

  companion object {
    val ambiguityThreshold = 0.7
    val targetLogTime = 0.05.seconds
    val cameraPoses = VisionConstants.CAMERA_TRANSFORMS

    val xyStdDevCoeffecient = 0.05
    val thetaStdDevCoefficient = 1.5
  }

  private val xyStdDevCoefficient = TunableNumber("Vision/xystdev", xyStdDevCoeffecient)

  private val thetaStdDev = TunableNumber("Vision/thetaStdDev", thetaStdDevCoefficient)

  private var fieldFramePoseSupplier = Supplier<Pose2d> { Pose2d() }
  private var visionConsumer: Consumer<List<TimestampedVisionUpdate>> = Consumer {}
  private var speakerVisionConsumer: Consumer<TimestampedTrigVisionUpdate> = Consumer {}
  private val lastFrameTimes = mutableMapOf<Int, Time>()
  private var lastDetectionTime = 0.0.seconds

  init {
    for (i in io.indices) {
      lastFrameTimes[i] = 0.0.seconds
    }
  }

  fun setDataInterfaces(
    fieldFramePoseSupplier: Supplier<Pose2d>,
    visionConsumer: Consumer<List<TimestampedVisionUpdate>>,
    speakerVisionMeasurementConsumer: Consumer<TimestampedTrigVisionUpdate>
  ) {
    this.fieldFramePoseSupplier = fieldFramePoseSupplier
    this.visionConsumer = visionConsumer
    this.speakerVisionConsumer = speakerVisionMeasurementConsumer
  }

  fun getShotConfidence(): Boolean {
    return Clock.realTimestamp - lastDetectionTime < 4.seconds &&
      trustedRobotDistanceToTarget < 210.inches
  }

  override fun periodic() {

    val startTime = Clock.realTimestamp

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
    }

    val robotPoses = mutableListOf<Pose2d?>()
    val robotDistancesToTarget = mutableListOf<Length?>()
    val visionUpdates = mutableListOf<TimestampedVisionUpdate>()

    for (instance in io.indices) {

      lastFrameTimes[instance] = Clock.fpgaTime
      val timestamp = inputs[instance].timestamp
      val values = inputs[instance].frame

      var cameraPose: Pose3d? = inputs[instance].frame
      var robotPose: Pose2d? = null
      var robotDistanceToTarget: Length? = null
      var tagTargets = inputs[instance].cameraTargets

      val cornerData = mutableListOf<Double>()

      for (tag in tagTargets) {
        if (DriverStation.getAlliance().isPresent) {
          if ((tag.fiducialId in intArrayOf(4) && !FMSData.isBlue) ||
            (tag.fiducialId in intArrayOf(7) && FMSData.isBlue)
          ) { // i made the tag IDS up

            robotDistanceToTarget =
              PhotonUtils.calculateDistanceToTargetMeters(
              cameraPoses[instance].translation.z.inMeters,
              57.125.inches.inMeters,
              24.02.degrees.inRadians,
              tag.pitch.degrees.inRadians
            )
              .meters + 4.inches

            lastDetectionTime = Clock.realTimestamp

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/robotDistanceToTarget",
              robotDistanceToTarget.inMeters
            )

            var cameraTspeaker2d =
              Translation2d(
                PhotonUtils.estimateCameraToTargetTranslation(
                  robotDistanceToTarget.inMeters, Rotation2d(-tag.yaw.degrees.inRadians)
                )
              )

            robotTSpeaker = Translation3d(cameraTspeaker2d.x, cameraTspeaker2d.y, 0.meters)

            val timestampedTrigVisionUpdate =
              TimestampedTrigVisionUpdate(
                inputs[instance].timestamp,
                Transform2d(Translation2d(robotTSpeaker.x, robotTSpeaker.y), 0.0.degrees)
              )
            speakerVisionConsumer.accept(timestampedTrigVisionUpdate)

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/robotTspeaker",
              robotTSpeaker.translation3d
            )

            for (corner in tag.detectedCorners) {
              cornerData.add(corner.x)
              cornerData.add(corner.y)
            }
          }
        }
      }

      Logger.recordOutput("Vision/cornerDetections/$instance}", cornerData.toDoubleArray())

      robotPoses.add(robotPose)
      robotDistancesToTarget.add(robotDistanceToTarget)

      if (cameraPose == null || robotPose == null) {
        continue
      }

      if ((robotPose.rotation - fieldFramePoseSupplier.get().rotation).absoluteValue > 7.degrees &&
        DriverStation.isEnabled()
      ) {
        continue
      }
    }

    var trustedRobotPose: Pose2d? = Pose2d()

    for (cameraInstance in VisionConstants.TRUSTED_CAMERA_ORDER) {
      if (cameraInstance in io.indices && robotPoses[cameraInstance] != null) {
        trustedRobotPose = robotPoses[cameraInstance]
        break
      }
    }

    for (cameraInstance in VisionConstants.TRUSTED_CAMERA_ORDER) {
      if (cameraInstance in io.indices && robotDistancesToTarget[cameraInstance] != null) {
        trustedRobotDistanceToTarget = robotDistancesToTarget[cameraInstance]!!
        break
      }
    }

    Logger.recordOutput(
      "LoggedRobot/VisionLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds
    )
  }
}
