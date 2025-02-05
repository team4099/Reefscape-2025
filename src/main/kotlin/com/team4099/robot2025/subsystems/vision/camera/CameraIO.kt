package com.team4099.robot2023.subsystems.vision.camera

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.common.dataflow.structures.Packet
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inDegrees

interface CameraIO {
  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var fps = 0.0
    var usedTargets: List<Int> = listOf<Int>()
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0
    var cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 })
    var distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *DoubleArray(5) { 0.0 })

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("frame", frame.pose3d)
      table?.put("fps", fps)
      table?.put("usedTargets", usedTargets.toIntArray())
      table?.put("cameraMatrix", cameraMatrix.data)
      table?.put("distCoeff", distCoeff.data)

      table?.put("cameraTargets/indices", cameraTargets.size)

      for (targetIndex in cameraTargets.indices) {
        table?.put("cameraTargets/$targetIndex/id", cameraTargets[targetIndex].fiducialId)
        table?.put("cameraTargets/$targetIndex/yaw", cameraTargets[targetIndex].yaw)
        table?.put("cameraTargets/$targetIndex/pitch", cameraTargets[targetIndex].pitch)
        table?.put("cameraTargets/$targetIndex/area", cameraTargets[targetIndex].area)
        table?.put("cameraTargets/$targetIndex/skew", cameraTargets[targetIndex].skew)
        for (i in 1 .. 4) {
          table?.put("cameraTargets/$targetIndex/corners/$i", cameraTargets[targetIndex].detectedCorners[i])
        }
        table?.put("cameraTargets/$targetIndex/cameraToTarget", cameraTargets[targetIndex].bestCameraToTarget)
        table?.put("cameraTargets/$targetIndex/ambiguity", cameraTargets[targetIndex].poseAmbiguity)
      }
    }

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", 0.0)?.let { timestamp = it.seconds }
      table?.get("frame", Pose3dWPILIB())?.let { frame = Pose3d(it.get(0)) }
      table?.get("fps", 0.0)
      table?.get("usedTargets", intArrayOf())?.let { usedTargets = it.toList() }

      table?.get("cameraTargets/indices", 0)?.let { indices = it }

      table?.get("distCoeff", MatBuilder.fill(Nat.N5(), Nat.N1(), *DoubleArray(5) { 0.0 }).data)
        ?.let { distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *it) }

      table?.get("cameraMatrix", MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 }).data)
        ?.let { cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *it) }

      cameraTargets = mutableListOf<PhotonTrackedTarget>()

      for (targetID in 0 until indices) {
        val target = PhotonTrackedTarget()

        target.fiducialId = table?.get("cameraTargets/$targetID/id", 0) ?: 0
        target.yaw = table?.get("cameraTarget/$targetID/yaw", 0.0) ?: 0.0
        target.pitch = table?.get("cameraTarget/$targetID/pitch", 0.0) ?: 0.0
        target.area = table?.get("cameraTarget/$targetID/area", 0.0) ?: 0.0
        target.pitch = table?.get("cameraTarget/$targetID/skew", 0.0) ?: 0.0
        val corners = mutableListOf<TargetCorner>()
        for (i in 1 .. 4) {
          val corner: TargetCorner? = table?.get("cameraTarget/$targetID/corners/$i", TargetCorner())
          corners.add(corner ?: TargetCorner())
        }
        target.detectedCorners = corners

        target.bestCameraToTarget = table?.get("cameraTarget/$targetID/cameraToTarget", Transform3d())?.get(0) ?: Transform3d()
        target.poseAmbiguity = table?.get("cameraTarget/$targetID/ambiguity", 0.0) ?: 0.0

        cameraTargets.add(target)
      }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}
