package com.team4099.robot2023.config.constants

import edu.wpi.first.apriltag.AprilTagFields
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.apriltag.AprilTagFieldLayout
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees

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

  val wpilibAprilTags =
    if (Constants.Universal.REAL_FIELD) aprilTags.map { it.apriltagWpilib }
    else homeAprilTags.map { it.apriltagWpilib }

  val wpilibFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      wpilibAprilTags, fieldLength.inMeters, fieldWidth.inMeters
    )

  var wingX = 229.201.inches
  var podiumX = 126.75.inches
  var startingLineX = 74.111.inches
  var subwooferX = 28.inches
  val edgeOfBumperToCenter = 12.75.inches + 3.5.inches

  val fieldAprilTags: List<AprilTag> =
    listOf(
      AprilTag(
        0,
        Pose3d(
          Translation3d(1.meters, 57.25.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        1,
        Pose3d(
          Translation3d(1.meters, 57.25.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        2,
        Pose3d(
          Translation3d(1.meters, 57.25.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        3,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches - 22.25.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        4,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        5,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        6,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        7,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        8,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        9,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        10,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        11,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        12,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        13,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        14,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        15,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      ),
      AprilTag(
        16,
        Pose3d(
          Translation3d(652.755.inches, 218.416.inches, 57.125.inches),
          Rotation3d(0.degrees, 0.degrees, 180.degrees)
        )
      )
    )

  val tags = AprilTagFields.k2024Crescendo

  var ampCenter = Translation2d(72.455.inches, 322.996.inches)

  var aprilTagWidth = 6.50.inches

  var noteThickness = 2.inches

  val bottomRightSpeaker = Pose2d(0.0.inches, 238.815.inches, 0.degrees)
  val bottomLeftSpeaker = Pose2d(0.0.inches, 197.765.inches, 0.degrees)
  val topRightSpeaker = Pose2d(18.055.inches, 238.815.inches, 0.degrees)
  val topLeftSpeaker = Pose2d(18.055.inches, 197.765.inches, 0.degrees)

  // Center of the speaker opening for the blue alliance
  val centerSpeakerOpening = bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5.seconds)

  /** Staging locations for each note */
  object StagingLocations {
    var centerlineX = fieldLength / 2.0

    // need to update
    var centerlineFirstY = 29.638.inches
    var centerlineSeparationY = 66.0.inches
    var spikeX = 114.0.inches

    // need
    var spikeFirstY = 161.638.inches
    var spikeSeparationY = 57.0.inches

    var centerlineTranslations: Array<Translation2d?> = arrayOfNulls(5)
    var spikeTranslations: Array<Translation2d?> = arrayOfNulls(3)

    init {
      for (i in centerlineTranslations.indices) {
        centerlineTranslations[i] =
          Translation2d(centerlineX, centerlineFirstY + (centerlineSeparationY * i))
      }
    }

    init {
      for (i in spikeTranslations.indices) {
        spikeTranslations[i] = Translation2d(spikeX, spikeFirstY + (spikeSeparationY * i))
      }
    }
  }

  /** Each corner of the speaker * */
  object Speaker {
    // corners (blue alliance origin)
    var topRightSpeaker = Translation3d(18.055.inches, 238.815.inches, 83.091.inches)

    var topLeftSpeaker = Translation3d(18.055.inches, 197.765.inches, 83.091.inches)

    var bottomRightSpeaker: Translation3d = Translation3d(0.0.inches, 238.815.inches, 78.324.inches)
    var bottomLeftSpeaker: Translation3d = Translation3d(0.0.inches, 197.765.inches, 78.324.inches)

    /** Center of the speaker opening (blue alliance) */
    var centerSpeakerOpening: Translation3d = (bottomLeftSpeaker + topRightSpeaker) / 2.0
    var speakerTargetPose = Pose2d(centerSpeakerOpening.x, centerSpeakerOpening.y, 0.0.degrees)
  }
}
