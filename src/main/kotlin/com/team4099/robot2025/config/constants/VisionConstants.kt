package com.team4099.robot2023.config.constants

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

  const val NUM_OF_CAMERAS = 2

  val TRUSTED_CAMERA_ORDER = arrayOf<Int>(1, 0)

  //  val CAMERA_TRANSFORMS =
  //    listOf(
  //      Transform3d(
  //        Translation3d(12.89.inches, 7.154.inches, 29.33.inches),
  //        Rotation3d(180.degrees, -5.degrees, 0.degrees)
  //      ),
  //      Transform3d(
  //        Translation3d(-9.98.inches, -6.64.inches, 16.508.inches),
  //        Rotation3d(0.0.degrees, 0.0.degrees, 180.degrees)
  //      ),
  //      Transform3d(
  //        Translation3d(-10.036.inches, -12.793.inches, 16.438.inches),
  //        Rotation3d(0.0.degrees, 0.0.degrees, -40.degrees)
  //      ), // camera facing rightward
  //      Transform3d(
  //        Translation3d(-10.560.inches, 12.793.inches, 16.437.inches),
  //        Rotation3d(180.0.degrees, 0.0.degrees, 40.degrees)
  //      ) // camera facing leftward
  //    )

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(12.653.inches, -9.1.inches, 14.25.inches), // 18.69
        Rotation3d(-5.3.degrees, 30.degrees, -72.77.degrees)
      ), // left
      Transform3d(
        Translation3d(4.8.inches, 0.inches, 17.164.inches), // 18.69
        Rotation3d(0.degrees, 30.degrees, 0.degrees)
      ), // front
    )

  val CAMERA_NAMES = listOf("parakeet_1", "parakeet_2", "parakeet_3")

  val robotTtag =
    Transform3d(
      Translation3d(
        102.6.inches + 25.75.inches,
        14.inches + 74.inches + 13.inches + 3.25.inches,
        19.25.inches + 1.meters - 3.25.inches
      ),
      Rotation3d(0.degrees, 0.degrees, 0.degrees)
    )

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
    const val RES_HEIGHT = 240 // no clue what these numbers should be but usnig these for now
  }
}
