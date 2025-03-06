package com.team4099.robot2025.config.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.wpilibj.Filesystem
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import java.nio.file.Path

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. **All units in Meters** <br></br> <br></br>
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall.<br></br> <br></br> Length refers to the *x* direction (as described by wpilib) <br></br>
 * Width refers to the *y* direction (as described by wpilib)
 */
object FieldConstants {

  val aprilTags: List<AprilTag> = listOf()
  val homeAprilTags: List<AprilTag> = listOf()

  val defaultAprilTagType: AprilTagLayoutType = AprilTagLayoutType.OFFICIAL

  enum class AprilTagLayoutType(name: String) {
    OFFICIAL("2025-official");

    val layout: org.team4099.lib.apriltag.AprilTagFieldLayout
    val layoutString: String

    init {

      val AprilTags = mutableListOf<AprilTag>()

      val wpiLayout =
        AprilTagFieldLayout(
          Path.of(Filesystem.getDeployDirectory().path, "apriltags", "$name.json")
        )

      for (tag in wpiLayout.tags) {
        AprilTags.add(AprilTag(tag.ID, Pose3d(tag.pose)))
      }

      layout =
        org.team4099.lib.apriltag.AprilTagFieldLayout(
          AprilTags, wpiLayout.fieldLength.meters, wpiLayout.fieldWidth.meters
        )
      layoutString = name
    }
  }

  val fieldLength = AprilTagLayoutType.OFFICIAL.layout.fieldLength
  val fieldWidth = AprilTagLayoutType.OFFICIAL.layout.fieldWidth

  object REEF {
    val blue_center: Translation2d = Translation2d(176.746.inches, fieldWidth / 2.0)
    val red_center: Translation2d = Translation2d(fieldLength - 176.746.inches, fieldWidth / 2.0)
  }
}
