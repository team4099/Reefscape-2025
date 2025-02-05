package com.team4099.robot2023.subsystems.vision.camera

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import java.util.Optional

class CameraIOPhotonvision(private val identifier: String, private val transform: Transform3dWPILIB) : CameraIO {

  private val photonEstimator: PhotonPoseEstimator
  private val camera: PhotonCamera
  private var lastEstTimestamp: Time = 0.0.seconds

  init {
    camera = PhotonCamera(identifier)

    photonEstimator =
      PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        transform
      )

    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
  }

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    if (camera.isConnected) {
      inputs.cameraMatrix = camera.cameraMatrix.get()
      inputs.distCoeff = camera.distCoeffs.get()
    }

    val pipelineResult = camera.latestResult

    Logger.recordOutput("$identifier/timestampIG", pipelineResult.timestampSeconds)
    Logger.recordOutput("$identifier/hasTarget", pipelineResult.hasTargets())

    if (pipelineResult.hasTargets()) {
      inputs.timestamp = pipelineResult.timestampSeconds.seconds
      inputs.cameraTargets = pipelineResult.targets

      if ((inputs.timestamp - lastEstTimestamp).absoluteValue > 10.micro.seconds) {
        inputs.fps = 1 / (inputs.timestamp - lastEstTimestamp).inSeconds
        lastEstTimestamp = inputs.timestamp
      }

      val visionEst: Optional<EstimatedRobotPose>? = photonEstimator.update(pipelineResult)
      if (visionEst != null && visionEst.isPresent) {
        val poseEst = Pose3d(visionEst.get().estimatedPose)
        inputs.usedTargets = visionEst.get().targetsUsed.map { it.fiducialId }
        inputs.frame = poseEst
      }
    }
  }
}
