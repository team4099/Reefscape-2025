package com.team4099.robot2025.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.toTransform3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonUtils
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
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
  private var reefVisionConsumer: Consumer<TimestampedTrigVisionUpdate> = Consumer {}
  private var closestReefTagAcrossCams: Map.Entry<Int, Pair<Int, Transform3d>?>? = null

  fun setDataInterfaces(
    fieldFramePoseSupplier: Supplier<Pose2d>,
    visionConsumer: Consumer<List<TimestampedVisionUpdate>>,
    reefVisionMeasurementConsumer: Consumer<TimestampedTrigVisionUpdate>
  ) {
    this.fieldFramePoseSupplier = fieldFramePoseSupplier
    this.visionConsumer = visionConsumer
    this.reefVisionConsumer = reefVisionMeasurementConsumer
  }

  var lastTrigVisionUpdate =
    TimestampedTrigVisionUpdate(Clock.fpgaTime, -1, Transform2d(Translation2d(), 0.degrees))

  override fun periodic() {

    val startTime = Clock.realTimestamp

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
    }

    val closestReefTags = mutableMapOf<Int, Pair<Int, Transform3d>?>()
    for (i in io.indices) {
      closestReefTags[i] = null
    }

    for (instance in io.indices) {

      var reefTags = mutableListOf<Pair<Int, Transform3d>>()
      var closestReefTag: Pair<Int, Transform3d>? = null

      var tagTargets = inputs[instance].cameraTargets

      val cornerData = mutableListOf<Double>()

      for (tag in tagTargets) {
        if (DriverStation.getAlliance().isPresent) {
          if ((tag.fiducialId in VisionConstants.BLUE_REEF_TAGS && FMSData.isBlue) ||
            (tag.fiducialId in VisionConstants.RED_REEF_TAGS && !FMSData.isBlue)
          ) {

            val cameraDistanceToTarget =
              PhotonUtils.calculateDistanceToTargetMeters(
                cameraPoses[instance].translation.z.inMeters,
                VisionConstants.REEF_TAG_HEIGHT.inMeters,
                cameraPoses[instance].rotation.y.inRadians,
                tag.pitch.degrees.inRadians
              )
                .meters

            var cameraTReefTag =
              Transform2d(
                PhotonUtils.estimateCameraToTarget(
                  PhotonUtils.estimateCameraToTargetTranslation(
                    cameraDistanceToTarget.inMeters,
                    Rotation2d(-tag.yaw.degrees.inRadians)
                  ),
                  FieldConstants.aprilTags.get(tag.fiducialId).pose.toPose2d().pose2d,
                  drivetrainOdometry().pose2d.rotation
                )
              )

            var robotTReefTag =
              Pose3d()
                .transformBy(cameraPoses[instance])
                .transformBy(
                  Transform3d(
                    Translation3d(
                      cameraTReefTag.translation.x,
                      cameraTReefTag.translation.y,
                      VisionConstants.REEF_TAG_HEIGHT -
                        cameraPoses[instance].translation.z
                    ),
                    Rotation3d(0.degrees, 0.degrees, cameraTReefTag.rotation)
                  )
                )
                .toTransform3d()

            val distanceToTarget = robotTReefTag.translation.norm

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/cameraDistanceToTarget",
              cameraDistanceToTarget.inMeters
            )

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotDistanceToTarget",
              distanceToTarget.inMeters
            )

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/cameraTReefTag",
              cameraTReefTag.transform2d
            )

            Logger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotTReefTag",
              robotTReefTag.transform3d
            )

            for (corner in tag.detectedCorners) {
              cornerData.add(corner.x)
              cornerData.add(corner.y)
            }

            reefTags.add(Pair(tag.fiducialId, robotTReefTag))
          }
        }
      }

      closestReefTag = reefTags.minByOrNull { it.second.translation.norm }

      closestReefTags[instance] = closestReefTag

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/cornerDetections}",
        cornerData.toDoubleArray()
      )

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestReefTagID}",
        closestReefTag?.first ?: -1
      )

      Logger.recordOutput(
        "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestReefTagPose}",
        closestReefTag?.second?.transform3d
      )
    }

    closestReefTagAcrossCams =
      closestReefTags.minByOrNull { it.value?.second?.translation?.norm ?: 1000000.meters }

    Logger.recordOutput(
      "Vision/ClosestReefTagAcrossAllCams/CameraID",
      if (closestReefTagAcrossCams != null)
        VisionConstants.CAMERA_NAMES[closestReefTagAcrossCams?.key ?: -1]
      else "None"
    )

    Logger.recordOutput(
      "Vision/ClosestReefTagAcrossAllCams/TagID", closestReefTagAcrossCams?.value?.first ?: -1
    )

    Logger.recordOutput(
      "Vision/ClosestReefTagAcrossAllCams/ReefTagPose",
      closestReefTagAcrossCams?.value?.second?.transform3d
    )

    if (closestReefTagAcrossCams?.key != null && closestReefTagAcrossCams?.value != null) {

      lastTrigVisionUpdate =
        TimestampedTrigVisionUpdate(
          inputs[closestReefTagAcrossCams?.key ?: 0].timestamp,
          closestReefTagAcrossCams?.value?.first ?: -1,
          Transform2d(
            Translation2d(
              closestReefTagAcrossCams?.value?.second?.translation?.x ?: 0.meters,
              closestReefTagAcrossCams?.value?.second?.translation?.y ?: 0.meters
            ),
            closestReefTagAcrossCams?.value?.second?.rotation?.z ?: 0.degrees
          )
        )

      reefVisionConsumer.accept(lastTrigVisionUpdate)
    }

    Logger.recordOutput(
      "LoggedRobot/VisionLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds
    )
  }
}
