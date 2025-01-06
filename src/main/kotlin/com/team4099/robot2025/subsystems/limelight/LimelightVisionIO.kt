package com.team4099.robot2025.subsystems.limelight

import com.team4099.robot2025.util.LimelightReading
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Decimal
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

interface LimelightVisionIO {

  class LimelightVisionIOInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var fps = 0.0
    var validReading = false
    var gamePieceTargets = listOf<LimelightReading>()
    var xAngle = 0.degrees
    var yAngle = 0.degrees
    var targetSize = 0.percent

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", timestamp.inSeconds)?.let { timestamp = it.seconds }
      table?.get("fps", fps)?.let { fps = it }
      table?.get("validReading", validReading)?.let { validReading = it }
      table?.get("xAngleDegrees", xAngle.inDegrees)?.let { xAngle = it.degrees }
      table?.get("yAngleDegrees", yAngle.inDegrees)?.let { yAngle = it.degrees }
      table?.get("targetSizePercent", targetSize.value)?.let { targetSize = it.percent }
      val numOfTargets = table?.get("numOfTargets", 0) ?: 0
      val retrievedTargets = mutableListOf<LimelightReading>()
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
            LimelightReading(
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

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("fps", fps)
      table?.put("validReading", validReading)
      table?.get("xAngleDegrees", xAngle.inDegrees)
      table?.get("yAngleDegrees", yAngle.inDegrees)
      table?.get("targetSizePercent", targetSize.value)
      table?.put("numOfTargets", gamePieceTargets.size.toLong())
      table?.put("cornersX", gamePieceTargets.map { it.txPixel }.toDoubleArray())
      table?.put("cornersY", gamePieceTargets.map { it.tyPixel }.toDoubleArray())
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

  fun updateInputs(inputs: LimelightVisionIOInputs) {}

  fun setPipeline(pipelineIndex: Int) {}

  fun setLeds(enabled: Boolean) {}
}
