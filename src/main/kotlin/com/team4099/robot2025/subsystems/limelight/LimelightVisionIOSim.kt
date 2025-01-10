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

class LimelightVisionIOSim(override val cameraName: String) : LimelightVisionIO {

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {}

  override fun setPipeline(pipelineIndex: Int) {}

  override fun setCameraPose(pose: Pose3d) {}

  override fun setTargetOffset(translation: Translation3d) {}

  override fun setLeds(enabled: Boolean) {}
}
