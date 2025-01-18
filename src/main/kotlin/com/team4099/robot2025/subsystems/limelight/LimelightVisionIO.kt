package com.team4099.robot2025.subsystems.limelight

import com.team4099.lib.vision.LimelightAprilTagReading
import com.team4099.lib.vision.LimelightNeuralDetectorReading
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Decimal
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

interface LimelightVisionIO {

  val cameraName: String

  class LimelightVisionIOInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var fps = 0.0
    var validReading = false
    var xAngle = 0.degrees
    var yAngle = 0.degrees
    var targetSize = 0.percent

    var gamePieceTargets = listOf<LimelightNeuralDetectorReading>()
    var aprilTagTargets = listOf<LimelightAprilTagReading>()
    var pipelineType = ""

    var primaryTagID = 0
    var cameraPoseTargetSpace = Pose3d()
    var targetPoseCameraSpace = Pose3d()
    var cameraPoseRobotSpace = Pose3d()
    var targetPoseRobotSpace = Pose3d()
    var robotPoseTargetSpace = Pose3d()
    var botPoseFieldSpace = Pose3d()

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", timestamp.inSeconds)?.let { timestamp = it.seconds }
      table?.get("fps", fps)?.let { fps = it }
      table?.get("validReading", validReading)?.let { validReading = it }
      table?.get("xAngleDegrees", xAngle.inDegrees)?.let { xAngle = it.degrees }
      table?.get("yAngleDegrees", yAngle.inDegrees)?.let { yAngle = it.degrees }
      table?.get("targetSizePercent", targetSize.value)?.let { targetSize = it.percent }
      table?.get("pipelineType", pipelineType)?.let { pipelineType = it }

      val numOfTargets = table?.get("numOfTargets", 0) ?: 0

      if (pipelineType == "pipe_fiducial") {
        val retrievedTargets = mutableListOf<LimelightAprilTagReading>()
        for (targetIndex in 0 until numOfTargets) {
          val tagID: Int? = table?.get("rawTags/$targetIndex/id", 0)
          val ambiguity: Double? = table?.get("rawTags/$targetIndex/ambiguity", 0.0)
          val targetTx: Angle? = table?.get("rawTags/$targetIndex/tx", 0.0)?.degrees
          val targetTy: Angle? = table?.get("rawTags/$targetIndex/ty", 0.0)?.degrees
          val distToCamera: Length? = table?.get("rawTags/$targetIndex/distToCamera", 0.0)?.meters
          val distToRobot: Length? = table?.get("rawTags/$targetIndex/distToRobot", 0.0)?.meters
          val targetTa: Decimal? = table?.get("rawTags/$targetIndex/ta", 0.0)?.percent
          if (tagID != null &&
            targetTx != null &&
            targetTy != null &&
            ambiguity != null &&
            distToCamera != null &&
            distToRobot != null &&
            targetTa != null
          ) {
            retrievedTargets.add(
              LimelightAprilTagReading(
                tagID, targetTx, targetTy, targetTa, distToCamera, distToRobot, ambiguity
              )
            )
          }
        }
        aprilTagTargets = retrievedTargets.toList()
      }

      if (pipelineType == "pipe_neuraldetector") {
        val retrievedTargets = mutableListOf<LimelightNeuralDetectorReading>()
        for (targetIndex in 0 until numOfTargets) {
          val className: String? = table?.get("Detection/$targetIndex/class", "")
          val confidence: Decimal? = table?.get("Detection/$targetIndex/conf", 0.0)?.percent
          val targetTx: Angle? = table?.get("Detection/$targetIndex/tx", 0.0)?.degrees
          val targetTy: Angle? = table?.get("Detection/$targetIndex/ty", 0.0)?.degrees
          val targetTxPixels: Double? = table?.get("Detection/$targetIndex/txp", 0.0)
          val targetTyPixels: Double? = table?.get("Detection/$targetIndex/typ", 0.0)
          val targetTa: Decimal? = table?.get("Detection/$targetIndex/ta", 0.0)?.percent
          if ((className == "cone" || className == "cube") &&
            confidence != null &&
            targetTx != null &&
            targetTy != null &&
            targetTxPixels != null &&
            targetTyPixels != null &&
            targetTa != null
          ) {
            retrievedTargets.add(
              LimelightNeuralDetectorReading(
                className,
                confidence,
                targetTx,
                targetTy,
                targetTxPixels,
                targetTyPixels,
                targetTa
              )
            )
          }
        }
        gamePieceTargets = retrievedTargets.toList()
      }
    }

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("fps", fps)
      table?.put("validReading", validReading)
      table?.put("xAngleDegrees", xAngle.inDegrees)
      table?.put("yAngleDegrees", yAngle.inDegrees)
      table?.put("targetSizePercent", targetSize.value)
      table?.put("pipelineType", pipelineType)

      if (pipelineType == "pipe_fiducial") {
        table?.put("numOfTargets", aprilTagTargets.size.toLong())

        for (i in aprilTagTargets.indices) {
          table?.put("rawTags/$i/id", aprilTagTargets[i].tagID)
          table?.put("rawTags/$i/ambiguity", aprilTagTargets[i].ambiguity)
          table?.put("rawTags/$i/tx", aprilTagTargets[i].tx.inDegrees)
          table?.put("rawTags/$i/ty", aprilTagTargets[i].ty.inDegrees)
          table?.put("rawTags/$i/distToCamera", aprilTagTargets[i].distToCamera.inMeters)
          table?.put("rawTags/$i/distToRobot", aprilTagTargets[i].distToRobot.inMeters)
          table?.put("rawTags/$i/ta", aprilTagTargets[i].ta.value)
        }
      }
      if (pipelineType == "pipe_neuraldetector") {
        table?.put("numOfTargets", gamePieceTargets.size.toLong())

        for (i in gamePieceTargets.indices) {
          table?.put("Detection/$i/class", gamePieceTargets[i].className)
          table?.put("Detection/$i/conf", gamePieceTargets[i].confidence.value)
          table?.put("Detection/$i/tx", gamePieceTargets[i].tx.inDegrees)
          table?.put("Detection/$i/ty", gamePieceTargets[i].ty.inDegrees)
          table?.put("Detection/$i/typ", gamePieceTargets[i].tyPixel)
          table?.put("Detection/$i/txp", gamePieceTargets[i].txPixel)
          table?.put("Detection/$i/ta", gamePieceTargets[i].ta.value)
        }
      }
    }
  }

  fun updateInputs(inputs: LimelightVisionIOInputs) {}

  fun setPipeline(pipelineIndex: Int) {}

  fun setCameraPose(pose: Pose3d) {}

  fun setTargetOffset(translation: Translation3d) {}

  fun setLeds(enabled: Boolean) {}
}
