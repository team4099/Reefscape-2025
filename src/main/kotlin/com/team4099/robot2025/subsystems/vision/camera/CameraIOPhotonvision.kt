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

class CameraIOPhotonvision(private val identifier: String) : CameraIO {

  private val photonEstimator: PhotonPoseEstimator
  private val camera: PhotonCamera
  private var lastEstTimestamp: Time = 0.0.seconds

  init {
    camera = PhotonCamera(identifier)

    photonEstimator =
      PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        Transform3dWPILIB()
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
    if (pipelineResult.hasTargets()) {
      inputs.timestamp = pipelineResult.timestampSeconds.seconds
      Logger.recordOutput("$identifier/hasTarget", pipelineResult.hasTargets())
      inputs.cameraTargets = pipelineResult.targets

      val visionEst: Optional<EstimatedRobotPose>? = photonEstimator.update()
      if (visionEst != null && visionEst.isPresent) {
        val poseEst = visionEst.get().let { Pose3d(it.estimatedPose) }
        val latestTimestamp = visionEst.get().timestampSeconds.seconds
        if ((latestTimestamp - lastEstTimestamp).absoluteValue > 10.micro.seconds) {
          inputs.fps = 1 / (latestTimestamp - lastEstTimestamp).inSeconds
          lastEstTimestamp = latestTimestamp
        }

        inputs.usedTargets = visionEst.get().targetsUsed.map { it.fiducialId }

        inputs.timestamp = lastEstTimestamp
        inputs.frame = poseEst
      }
    }
  }
}
