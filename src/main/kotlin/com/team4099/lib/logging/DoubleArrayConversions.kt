package com.team4099.lib.logging

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Quaternion
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.geometry.Twist3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.derived.inRadians

fun Pose2d.toDoubleArray(): Array<Double> {
  return arrayOf(this.x.inMeters, this.y.inMeters, this.rotation.inRadians)
}

fun Pose3d.toDoubleArray(): Array<Double> {
  return arrayOf(
    this.x.inMeters,
    this.y.inMeters,
    this.z.inMeters,
    this.rotation.x.inRadians,
    this.rotation.y.inRadians,
    this.rotation.z.inRadians
  )
}

fun Quaternion.toDoubleArray(): Array<Double> {
  return arrayOf(this.w.inRadians, this.x.inMeters, this.y.inMeters, this.z.inMeters)
}

fun Rotation3d.toDoubleArray(): Array<Double> {
  return arrayOf(this.x.inRadians, this.y.inRadians, this.z.inRadians)
}

fun Transform2d.toDoubleArray(): DoubleArray {
  return doubleArrayOf(
    this.translation.x.inMeters, this.translation.y.inMeters, this.rotation.inRadians
  )
}

fun Transform3d.toDoubleArray(): Array<Double> {
  return arrayOf(
    this.translation.x.inMeters,
    this.translation.y.inMeters,
    this.translation.z.inMeters,
    this.rotation.x.inRadians,
    this.rotation.y.inRadians,
    this.rotation.z.inRadians
  )
}

fun Translation2d.toDoubleArray(): Array<Double> {
  return arrayOf(this.x.inMeters, this.y.inMeters)
}

fun Translation3d.toDoubleArray(): Array<Double> {
  return arrayOf(this.x.inMeters, this.y.inMeters, this.z.inMeters)
}

fun Twist2d.toDoubleArray(): Array<Double> {
  return arrayOf(this.dx.inMeters, this.dy.inMeters, this.dtheta.inRadians)
}

fun Twist3d.toDoubleArray(): Array<Double> {
  return arrayOf(
    this.dx.inMeters,
    this.dy.inMeters,
    this.dz.inMeters,
    this.rx.inRadians,
    this.ry.inRadians,
    this.rz.inRadians
  )
}

fun SwerveModulePosition.toDoubleArray(): Array<Double> {
  return arrayOf(this.distanceMeters, this.angle.radians)
}

fun SwerveModuleState.toDoubleArray(): Array<Double> {
  return arrayOf(this.speedMetersPerSecond, this.angle.radians)
}
