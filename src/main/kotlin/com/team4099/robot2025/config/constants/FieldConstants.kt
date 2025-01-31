// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package com.team4099.robot2025.config.constants

import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import java.io.IOException
import java.nio.file.Path

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
object FieldConstants {
  val fieldLength: Length = 690.876.inches
  val fieldWidth: Length = 317.0.inches
  val startingLineX: Length = 299.438.inches // Measured from the inside of starting line
  val algaeDiameter: Length = 16.0.inches
  val aprilTagWidth: Length = 6.50.inches
  val aprilTagCount: Int = 22

  object Processor {
    val centerFace: Pose2d = Pose2d(235.726.inches, 0.0.inches, 90.0.degrees)
  }

  object Barge {
    val farCage: Translation2d = Translation2d(345.428.inches, 286.779.inches)
    val middleCage: Translation2d = Translation2d(345.428.inches, 242.855.inches)
    val closeCage: Translation2d = Translation2d(345.428.inches, 199.947.inches)

    // Measured from floor to bottom of cage
    val deepHeight: Double = Units.inchesToMeters(3.125)
    val shallowHeight: Double = Units.inchesToMeters(30.125)
  }

  object CoralStation {
    val leftCenterFace: Pose2d = Pose2d(
      33.526.inches,
      291.176.inches,
      (90 - 144.011).degrees
    )
    val rightCenterFace: Pose2d = Pose2d(
      33.526.inches,
      25.824.inches,
      (144.011 - 90).degrees
    )
  }

  object Reef {
    val center: Translation2d = Translation2d(176.746.inches, 158.501.inches)
    val faceToZoneLine: Double = Units.inchesToMeters(12.0) // Side of the reef to the inside of the reef zone line
    val centerFaces: MutableList<Pose2d?> = mutableListOf<Pose2d?>() // Starting facing the driver station in clockwise order
    val branchPositions: MutableList<Map<ReefHeight, Pose3d>> =
      ArrayList() // Starting at the right branch facing the driver station in clockwise

    init {
      // Initialize faces
      centerFaces.addAll(
        mutableListOf<Pose2d?>(
        Pose2d(
          144.003.inches,
          158.500.inches,
          180.0.degrees
        ),
        Pose2d(
          160.373.inches,
          186.857.inches,
          120.0.degrees
        ),
        Pose2d(
          193.116.inches,
          186.858.inches,
          60.0.degrees
        ),
        Pose2d(
          209.489.inches,
          158.502.inches,
          0.0.degrees
        ),
        Pose2d(
          193.118.inches,
          130.145.inches,
          -60.0.degrees
        ),
        Pose2d(
          160.375.inches,
          130.144.inches,
          -120.0.degrees
        ))
      )

      // Initialize branch positions
      for (face in 0..5) {
        val fillRight: MutableMap<ReefHeight, Pose3d> = HashMap()
        val fillLeft: MutableMap<ReefHeight, Pose3d> = HashMap()
        for (level: ReefHeight in ReefHeight.values()) {
          val poseDirection: Pose2d = Pose2d(center, (180 - (60 * face)).degrees)
          val adjustX: Length = 30.738.inches
          val adjustY: Length = 6.469.inches
          fillRight[level] = Pose3d(
            Translation3d(
              poseDirection
                .transformBy(Transform2d(Translation2d(adjustX, adjustY), 0.degrees))
                .x,
              poseDirection
                .transformBy(Transform2d(Translation2d(adjustX, adjustY), 0.degrees))
                .y,
              level.height
            ),
            Rotation3d(
              0.0.degrees,
              level.pitch,
              poseDirection.rotation
            )
          )
          fillLeft[level] = Pose3d(
            Translation3d(
              poseDirection
                .transformBy(Transform2d(Translation2d(adjustX, -adjustY), 0.degrees))
                .x,
              poseDirection
                .transformBy(Transform2d(Translation2d(adjustX, -adjustY), 0.degrees))
                .y,
              level.height
            ),
            Rotation3d(
              0.0.degrees,
              level.pitch,
              poseDirection.rotation
            )
          )
        }
        branchPositions.add(fillRight)
        branchPositions.add(fillLeft)
      }
    }
  }

  object StagingPositions {
    // Measured from the center of the ice cream
    val leftIceCream: Pose2d = Pose2d(48.0.inches, 230.5.inches, 0.degrees)
    val middleIceCream: Pose2d = Pose2d(48.0.inches, 158.5.inches, 0.degrees)
    val rightIceCream: Pose2d = Pose2d(48.0.inches, 86.5.inches, 0.degrees)
  }

  enum class ReefHeight // in degrees
    (val height: Length, val pitch: Angle) {
    L4(72.0.inches, -90.degrees),
    L3(47.625.inches, -35.degrees),
    L2(31.875.inches, -35.degrees),
    L1(18.0.inches, 0.degrees)
  }

  object AprilTagLayout {

  }

}