package com.team4099.robot2025.config.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.wpilibj.Filesystem
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.inches
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
  var fieldLength = 651.223.inches
  var fieldWidth = 323.277.inches

  val aprilTags: List<AprilTag> = listOf()
  val homeAprilTags: List<AprilTag> = listOf()

  val defaultAprilTagType: AprilTagLayoutType = AprilTagLayoutType.OFFICIAL

  enum class AprilTagLayoutType(name: String) {
    OFFICIAL("2025-official");

    private val layout: org.team4099.lib.apriltag.AprilTagFieldLayout
    private val layoutString: String

    init {

      val AprilTags = mutableListOf<AprilTag>()

      for (tag in AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().path, "apriltags", "$name.json")).tags) {
        AprilTags.add(AprilTag(tag.ID, Pose3d(tag.pose)))
      }

      layout = org.team4099.lib.apriltag.AprilTagFieldLayout(AprilTags, fieldLength, fieldWidth)
      layoutString = name
    }
  }



}
