package com.team4099.lib.math

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians

/**
 * Multiplies a twist by a scaling factor
 *
 * @param twist The twist to multiply
 * @param factor The scaling factor for the twist components
 * @return The new twist
 */
fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d {
  return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
}

/**
 * Translates a pose with respect to its own angle. Takes care of translation specific rotations
 * necessary.
 * @param translation2d The pure translation that is being applied. Note this will be applied in the
 * pose's axis.
 * @return Translated pose
 */
fun Pose2d.purelyTranslateBy(translation2d: Translation2d): Pose2d {
  return this.transformBy(Transform2d(translation2d.rotateBy(-this.rotation), 0.0.degrees))
}

/**
 * Returns the transform between the frame origin of the pose and the current pose state -- for
 * example, if the pose describes the pose of the robot in the odometry frame, the returned
 * transform will be the transform between the odometry frame and the robot frame.
 * @return
 */
fun Pose2d.asTransform2d(): Transform2d {
  return Transform2d(Pose2d(0.meters, 0.meters, 0.radians), this)
}

/**
 * Returns pose of whatever the transform represents, in the original frame of the transform. For
 * example, odomTRobot.asPose2d() would give the pose of the robot in the odometry frame.
 */
fun Transform2d.asPose2d(): Pose2d {
  return Pose2d(this.translation, this.rotation)
}
