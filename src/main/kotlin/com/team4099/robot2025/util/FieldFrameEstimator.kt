package com.team4099.robot2025.util

import com.team4099.lib.hal.Clock
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import java.util.NavigableMap
import java.util.TreeMap
import kotlin.Comparator
import kotlin.collections.ArrayList

class FieldFrameEstimator(stateStdDevs: Matrix<N3?, N1?>) {
  // Maintains the state of the field frame transform for the period of time before the currently
  // tracked history
  private var baseOdometryTField: Transform2d =
    Transform2d(Translation2d(0.meters, 0.meters), 0.radians)

  // Maintains the latest state of the field frame transform, including the currently tracked
  // history
  private var odometryTField: Transform2d =
    Transform2d(Translation2d(0.meters, 0.meters), 0.radians)

  private var odometryTReef: Transform2d = Transform2d(Translation2d(0.meters, 0.meters), 0.radians)

  private val updates: NavigableMap<Time, PoseUpdate> = TreeMap()
  private val q: Matrix<N3?, N1?> = Matrix(Nat.N3(), Nat.N1())

  /** Returns the latest robot pose based on drive and vision data. */
  fun getLatestOdometryTField(): Transform2d {
    return odometryTField
  }

  /** Returns the latest reef pose in the odometry frame. */
  fun getLatestOdometryTReef(): Transform2d {
    return odometryTReef
  }

  /** Resets the field frame transform to a known pose. */
  fun resetFieldFrameFilter(transform: Transform2d) {
    baseOdometryTField = transform
    updates.clear()
    update()
  }

  /** Records a new drive movement. */
  fun addDriveData(timestamp: Time, odomTRobot: Pose2d) {
    updates[timestamp] = PoseUpdate(odomTRobot, ArrayList<VisionUpdate>())
    update()
  }

  fun addReefVisionData(visionData: TimestampedTrigVisionUpdate) {
    val timestamp: Time = visionData.timestamp
    if (updates.containsKey(timestamp)) {
      // There was already an odometry update at this timestamp, add to it
      val odomTRobotAtVisionTimestamp = updates[timestamp]!!.odomTRobot
      val robotTReef = visionData.robotTReefTag
      odometryTReef = odomTRobotAtVisionTimestamp.transformBy(robotTReef).asTransform2d()
    } else {
      // Insert a new update
      val prevUpdate = updates.floorEntry(timestamp)
      val nextUpdate = updates.ceilingEntry(timestamp)
      if (prevUpdate == null || nextUpdate == null) {
        // Outside the range of existing data
        return
      }

      // Create partial twists (prev -> vision, vision -> next)
      val prevToVisionTwist =
        multiplyTwist(
          prevUpdate.value.odomTRobot.log(nextUpdate.value.odomTRobot),
          (timestamp - prevUpdate.key) / (nextUpdate.key - prevUpdate.key)
        )

      // Add new pose updates
      val odomTRobotAtVisionTimestamp = prevUpdate.value.odomTRobot.exp(prevToVisionTwist)
      val robotTSpeaker = visionData.robotTReefTag
      odometryTReef = odomTRobotAtVisionTimestamp.transformBy(robotTSpeaker).asTransform2d()
    }
  }

  /** Records a new set of vision updates. */
  fun addVisionData(visionData: List<TimestampedVisionUpdate>) {

    for (timestampedVisionUpdate in visionData) {
      val timestamp: Time = timestampedVisionUpdate.timestamp
      val visionUpdate =
        VisionUpdate(
          timestampedVisionUpdate.fieldTRobot,
          timestampedVisionUpdate.stdDevs,
          timestampedVisionUpdate.fromVision
        )
      if (updates.containsKey(timestamp)) {

        // There was already an update at this timestamp, add to it
        val oldVisionUpdates: ArrayList<VisionUpdate> = updates[timestamp]!!.visionUpdates
        oldVisionUpdates.add(visionUpdate)
        oldVisionUpdates.sortWith(VisionUpdate.compareDescStdDev)
      } else {

        // Insert a new update
        val prevUpdate = updates.floorEntry(timestamp)
        val nextUpdate = updates.ceilingEntry(timestamp)
        if (prevUpdate == null || nextUpdate == null) {
          // Outside the range of existing data
          return
        }

        // Create partial twists (prev -> vision, vision -> next)
        val prevToVisionTwist =
          multiplyTwist(
            prevUpdate.value.odomTRobot.log(nextUpdate.value.odomTRobot),
            (timestamp - prevUpdate.key) / (nextUpdate.key - prevUpdate.key)
          )

        // Add new pose updates
        val newVisionUpdates = ArrayList<VisionUpdate>()
        newVisionUpdates.add(visionUpdate)
        newVisionUpdates.sortWith(VisionUpdate.compareDescStdDev)
        updates[timestamp] =
          PoseUpdate(prevUpdate.value.odomTRobot.exp(prevToVisionTwist), newVisionUpdates)
      }
    }

    // Recalculate latest pose once
    update()
  }

  /** Clears old data and calculates the latest pose. */
  private fun update() {
    // Clear old data and update base pose
    // NOTE(parth): We need to maintain the history so that when vision updates come in, they have
    // some buffer to interpolate within.
    while (updates.size > 1 && updates.firstKey() < Clock.fpgaTime - HISTORY_LENGTH) {
      val (_, value) = updates.pollFirstEntry()
      baseOdometryTField = value.apply(baseOdometryTField, q)
    }

    // Update latest pose
    odometryTField = baseOdometryTField
    for (updateEntry in updates.entries) {
      odometryTField = updateEntry.value.apply(odometryTField, q)
    }

    for (update in updates) {
      if (update.value.visionUpdates.size > 0 && update.value.visionUpdates[0].fromVision) {
        Logger.recordOutput("Vision/Buffer/Vision", update.key.inSeconds)

        Logger.recordOutput(
          "Vision/Buffer/VisionPose",
          Pose2dWPILIB.struct,
          update.value.visionUpdates[0].fieldTRobot.pose2d
        )
      } else {
        Logger.recordOutput("Vision/Buffer/Drivetrain", update.key.inSeconds)
      }
    }
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
   * vision updates.
   */
  private class PoseUpdate(val odomTRobot: Pose2d, val visionUpdates: ArrayList<VisionUpdate>) {
    fun apply(previousOdomTField: Transform2d, q: Matrix<N3?, N1?>): Transform2d {
      var currentOdomTField = previousOdomTField

      // Apply vision updates
      for (visionUpdate in visionUpdates) {
        // Calculate Kalman gains based on std devs
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        val visionK: Matrix<N3, N3> = Matrix(Nat.N3(), Nat.N3())
        val r = DoubleArray(3)
        for (i in 0..2) {
          r[i] = visionUpdate.stdDevs.get(i, 0) * visionUpdate.stdDevs.get(i, 0)
        }
        for (row in 0..2) {
          if (q.get(row, 0) === 0.0) {
            visionK.set(row, row, 0.0)
          } else {
            visionK.set(
              row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row]))
            )
          }
        }

        // Calculate odom_T_field from this update's vision pose
        val odomTVisionField =
          odomTRobot.asTransform2d() + visionUpdate.fieldTRobot.asTransform2d().inverse()

        // Calculate twist between current field frame transform and latest vision update
        val fieldTVisionField = currentOdomTField.inverse() + odomTVisionField
        val visionTwist = fieldTVisionField.log()

        // Multiply by Kalman gain matrix
        val twistMatrix =
          visionK.times(
            VecBuilder.fill(
              visionTwist.dx.inMeters, visionTwist.dy.inMeters, visionTwist.dtheta.inRadians
            )
          )

        // Apply twist
        currentOdomTField +=
          Transform2d.exp(
            Twist2d(
              twistMatrix.get(0, 0).meters,
              twistMatrix.get(1, 0).meters,
              twistMatrix.get(2, 0).radians
            )
          )
      }
      return currentOdomTField
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  class VisionUpdate(
    val fieldTRobot: Pose2d,
    val stdDevs: Matrix<N3, N1>,
    val fromVision: Boolean = false
  ) {
    companion object {
      val compareDescStdDev = Comparator { a: VisionUpdate, b: VisionUpdate ->
        -(a.stdDevs.get(0, 0) + a.stdDevs.get(1, 0)).compareTo(
          b.stdDevs.get(0, 0) + b.stdDevs.get(1, 0)
        )
      }
    }
  }

  companion object {
    private val HISTORY_LENGTH = 0.3.seconds
  }

  init {
    for (i in 0..2) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0))
    }
  }
}
