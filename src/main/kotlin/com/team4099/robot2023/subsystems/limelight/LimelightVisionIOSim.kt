package com.team4099.robot2023.subsystems.limelight

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.VisionConstants
import com.team4099.robot2023.util.LimelightReading
import com.team4099.robot2023.util.rotateBy
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sqrt

object LimelightVisionIOSim : LimelightVisionIO {

  var poseSupplier: () -> Pose2d = { Pose2d() }

  override fun updateInputs(inputs: LimelightVisionIO.LimelightVisionIOInputs) {
    inputs.timestamp = Clock.realTimestamp
    inputs.xAngle = 0.0.radians
    inputs.yAngle = 0.0.radians
    inputs.targetSize = 0.0.percent
    inputs.fps = 90.0
    inputs.validReading = true

    inputs.gamePieceTargets =
      listOf(
        LimelightReading("cone", 0.0.percent, 0.0.degrees, 0.0.degrees, 0.0, 0.0, 0.0.percent)
      )
  }

  private fun cartesianToSpherical(translationInCameraSpace: Translation3d): Rotation3d {
    val x = translationInCameraSpace.x
    val y = translationInCameraSpace.y
    val z = translationInCameraSpace.z

    val r = sqrt(x.inMeters.pow(2) + y.inMeters.pow(2) + z.inMeters.pow(2)).meters
    val theta = acos(z / r).radians.rotateBy(-VisionConstants.Limelight.LL_TRANSFORM.rotation.y)
    val phi = (y.sign * acos(x.inMeters / sqrt(x.inMeters.pow(2) + y.inMeters.pow(2)))).radians

    return Rotation3d(phi, theta, 0.0.degrees)
  }
}
