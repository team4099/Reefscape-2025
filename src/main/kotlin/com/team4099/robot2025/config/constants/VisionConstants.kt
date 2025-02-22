package com.team4099.robot2025.config.constants

import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import kotlin.math.tan

object VisionConstants {
  const val SIM_POSE_TOPIC_NAME = "Odometry/groundTruthPose"
  const val POSE_TOPIC_NAME = "Odometry/pose"

  const val NUM_OF_CAMERAS = 2

  val BLUE_REEF_TAGS = arrayOf(17, 18, 19, 20, 21, 22)
  val RED_REEF_TAGS = arrayOf(6, 7, 8, 9, 10, 11)

  val BLUE_REEF_TAG_GYRO_ALIGNMENTS =
    mapOf(
      17 to 0.degrees,
      18 to 0.degrees,
      19 to 0.degrees,
      20 to 0.degrees,
      21 to 60.degrees,
      22 to 120.degrees
    )

  val RED_REEF_TAG_GYRO_ALIGNMENTS =
    mapOf(
      6 to 0.degrees,
      6 to 0.degrees,
      8 to 0.degrees,
      9 to 0.degrees,
      10 to 0.degrees,
      11 to 0.degrees
    )

  val TRUSTED_CAMERA_ORDER = arrayOf<Int>(1, 0)

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(10.49.inches, 12.325.inches, 8.195.inches), // 18.69
        Rotation3d(0.0.degrees, -20.degrees, -30.degrees)
      ), // l
      Transform3d(
        Translation3d(10.49.inches, -12.325.inches, 8.195.inches), // 18.69
        Rotation3d(0.0.degrees, -20.degrees, 30.degrees)
      )
    )

  val REEF_TAG_HEIGHT = 12.inches

  val CAMERA_NAMES = listOf("raven_1", "raven_2")

  object CAMERA_OV2387 {
    val CAMERA_PX = 1600
    val CAMERA_PY = 1200

    val HORIZONTAL_FOV = 80.degrees // i made these up lol
    val VERTICAL_FOV = 64.25.degrees

    val vpw = 2.0 * tan(HORIZONTAL_FOV.inRadians / 2)
    val vph = 2.0 * tan(VERTICAL_FOV.inRadians / 2)
  }

  object Limelight {
    val LIMELIGHT_NAME = "limelight-owl"
    val HORIZONTAL_FOV = 59.6.degrees
    val VERITCAL_FOV = 45.7.degrees
    val HIGH_TAPE_HEIGHT = 43.875.inches + 1.inches
    val MID_TAPE_HEIGHT = 23.905.inches + 1.inches
    val LL_TRANSFORM =
      Transform3d(
        Translation3d(-14.655.inches, 0.inches, 23.316.inches),
        Rotation3d(0.degrees, 143.degrees, 180.degrees)
      )
    const val RES_WIDTH = 320
    const val RES_HEIGHT = 240
  }
}
