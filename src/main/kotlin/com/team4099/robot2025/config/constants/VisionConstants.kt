package com.team4099.robot2025.config.constants

import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import kotlin.math.tan

object VisionConstants {
  const val FRONT_CAMERA_NAME = "limelight"
  const val BACK_CAMERA_NAME = "thing2"
  const val SIDE_CAMERA_NAME = "thing3"

  const val SIM_POSE_TOPIC_NAME = "Odometry/groundTruthPose"
  const val POSE_TOPIC_NAME = "Odometry/pose"

  // Define the CameraDetailModel interface
  interface CameraDetailModel {
    var cameraIntrinsics: CameraIntrinsics
    var cameraName: String
    var cameraTransform: Transform3d
  }

  // Implement the CameraDetailModel with a data class
  data class CameraDetailModelImpl(
    override var cameraIntrinsics: CameraIntrinsics,
    override var cameraName: String,
    override var cameraTransform: Transform3d
  ) : CameraDetailModel

  // Create a list of CameraDetailModel instances
  val cameraDetails = listOf(
    CameraDetailModelImpl(
      cameraIntrinsics = CameraIntrinsics.CAMERA_OV2387,
      cameraName = "Raven_One",
      cameraTransform = Transform3d()
    ),
    CameraDetailModelImpl(
      cameraIntrinsics = CameraIntrinsics.Limelight,
      cameraName = "Raven_Two",
      cameraTransform = Transform3d()
    )
  )

}

sealed class CameraIntrinsics {
  object CAMERA_OV2387 : CameraIntrinsics() {
    const val CAMERA_PX = 1600
    const val CAMERA_PY = 1200

    val HORIZONTAL_FOV = 80.0.degrees
    val VERTICAL_FOV = 64.25.degrees

    val vpw: Double get() = 2.0 * tan(HORIZONTAL_FOV.inRadians / 2)
    val vph: Double get() = 2.0 * tan(VERTICAL_FOV.inRadians / 2)
  }

  object Limelight : CameraIntrinsics() {
    const val LIMELIGHT_NAME = "limelight-owl"
    val HORIZONTAL_FOV = 59.6.degrees
    val VERTICAL_FOV = 45.7.degrees

    const val RES_WIDTH = 320
    const val RES_HEIGHT = 240
  }
}