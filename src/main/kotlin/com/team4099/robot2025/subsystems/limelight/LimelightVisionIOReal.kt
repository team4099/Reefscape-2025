package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.lib.vision.LimelightAprilTagReading
import com.team4099.lib.vision.LimelightNeuralDetectorReading
import com.team4099.robot2025.subsystems.limelight.LimelightVisionIO
import com.team4099.utils.LimelightHelpers
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.milli

class LimelightVisionIOReal(override val cameraName: String) : LimelightVisionIO {

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    val totalLatency =
      LimelightHelpers.getLatency_Pipeline(cameraName).milli.seconds +
        LimelightHelpers.getLatency_Capture(cameraName).milli.seconds

    inputs.timestamp = Clock.realTimestamp - totalLatency
    inputs.xAngle = LimelightHelpers.getTX(cameraName).degrees
    inputs.yAngle = LimelightHelpers.getTY(cameraName).degrees
    inputs.targetSize = LimelightHelpers.getTA(cameraName).percent
    inputs.fps = 1000 / totalLatency.inMilliseconds
    inputs.validReading = LimelightHelpers.getTV(cameraName)
    inputs.pipelineType = LimelightHelpers.getCurrentPipelineType(cameraName)

    if (inputs.pipelineType == "pipe_fiducial") {
      inputs.aprilTagTargets =
        LimelightHelpers.getRawFiducials(cameraName).map { LimelightAprilTagReading(it) }

      inputs.primaryTagID = LimelightHelpers.getFiducialID(cameraName).toInt()
      inputs.cameraPoseTargetSpace =
        Pose3d(LimelightHelpers.getCameraPose3d_TargetSpace(cameraName))
      inputs.targetPoseCameraSpace =
        Pose3d(LimelightHelpers.getTargetPose3d_CameraSpace(cameraName))
      inputs.robotPoseTargetSpace = Pose3d(LimelightHelpers.getBotPose3d_TargetSpace(cameraName))
      inputs.cameraPoseRobotSpace = Pose3d(LimelightHelpers.getCameraPose3d_RobotSpace(cameraName))
      inputs.targetPoseRobotSpace = Pose3d(LimelightHelpers.getTargetPose3d_RobotSpace(cameraName))
      inputs.botPoseFieldSpace = Pose3d(LimelightHelpers.getBotPose3d_wpiBlue(cameraName))

      Logger.recordOutput("LimelightVision/$cameraName/primaryTag/id", inputs.primaryTagID)
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/cameraPoseTargetSpace",
        inputs.cameraPoseTargetSpace.pose3d
      )
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/targetPoseCameraSpace",
        inputs.targetPoseCameraSpace.pose3d
      )
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/robotPoseTargetSpace",
        inputs.robotPoseTargetSpace.pose3d
      )
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/cameraPoseRobotSpace",
        inputs.cameraPoseRobotSpace.pose3d
      )
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/targetPoseRobotSpace",
        inputs.targetPoseRobotSpace.pose3d
      )
      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/botPoseFieldSpace",
        inputs.botPoseFieldSpace.pose3d
      )

      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/robotToTagDistance",
        (inputs.targetPoseRobotSpace.translation.norm.inMeters)
      )

      Logger.recordOutput(
        "LimelightVision/$cameraName/primaryTag/robotToTagDistanceBotPose",
        (inputs.robotPoseTargetSpace.translation.norm.inMeters)
      )
    }

    if (inputs.pipelineType == "pipe_neuraldetector") {
      // Parsing JSON is slow(~2ms) according to LL docs, should replace with RawDetections if we
      // need to reduce loop times
      inputs.gamePieceTargets =
        LimelightHelpers.getLatestResults(cameraName).targets_Detector.map {
          LimelightNeuralDetectorReading(it)
        }
    }
  }

  override fun setPipeline(pipelineIndex: Int) {
    LimelightHelpers.setPipelineIndex(cameraName, pipelineIndex)
  }

  override fun setLeds(enabled: Boolean) {
    if (enabled) LimelightHelpers.setLEDMode_ForceOn(cameraName)
    else LimelightHelpers.setLEDMode_ForceOff(cameraName)
  }

  override fun setCameraPose(pose: Pose3d) {
    LimelightHelpers.setCameraPose_RobotSpace(
      cameraName,
      pose.translation.x.inMeters,
      pose.translation.y.inMeters,
      pose.translation.z.inMeters,
      pose.rotation.x.inDegrees,
      pose.rotation.y.inDegrees,
      pose.rotation.z.inDegrees
    )
  }

  override fun setTargetOffset(translation: Translation3d) {
    LimelightHelpers.setFiducial3DOffset(
      cameraName, translation.x.inMeters, translation.y.inMeters, translation.z.inMeters
    )
  }
}
