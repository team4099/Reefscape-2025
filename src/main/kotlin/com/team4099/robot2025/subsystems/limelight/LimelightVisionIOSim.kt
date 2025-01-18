package com.team4099.robot2025.subsystems.limelight

import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation3d

class LimelightVisionIOSim(override val cameraName: String) : LimelightVisionIO {

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {}

  override fun setPipeline(pipelineIndex: Int) {}

  override fun setCameraPose(pose: Pose3d) {}

  override fun setTargetOffset(translation: Translation3d) {}

  override fun setLeds(enabled: Boolean) {}
}
