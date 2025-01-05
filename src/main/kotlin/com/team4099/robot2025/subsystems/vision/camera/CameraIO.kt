package com.team4099.robot2023.subsystems.vision.camera

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.common.dataflow.structures.Packet
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.PhotonTrackedTarget.APacketSerde
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

interface CameraIO {
  class CameraInputs : LoggableInputs {
    var timestamp = 0.0.seconds
    var frame: Pose3d = Pose3d()
    var fps = 0.0
    var usedTargets: List<Int> = listOf<Int>()
    var cameraTargets = mutableListOf<PhotonTrackedTarget>()
    var indices = 0
    val photonPacketSerde = APacketSerde()
    var cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 })
    var distCoeff = MatBuilder.fill(Nat.N5(), Nat.N1(), *DoubleArray(5) { 0.0 })

    override fun toLog(table: LogTable?) {
      table?.put("timestampSeconds", timestamp.inSeconds)
      table?.put("frame", frame.pose3d)
      table?.put("fps", fps)
      table?.put("usedTargets", usedTargets.toIntArray())
      table?.put("cameraMatrix", cameraMatrix.data)
      table?.put("distCoeff", distCoeff.data)

      for (targetID in cameraTargets.indices) {
        val photonPacket = Packet(photonPacketSerde.maxByteSize)
        photonPacketSerde.pack(photonPacket, cameraTargets.get(targetID))
        table?.put("cameraTargets/$targetID", photonPacket.data)
      }

      table?.put("cameraTargets/indices", cameraTargets.size)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("timestampSeconds", 0.0)?.let { timestamp = it.seconds }
      table?.get("frame", Pose3dWPILIB())?.let { frame = Pose3d(it.get(0)) }
      table?.get("fps", 0.0)
      table?.get("usedTargets", intArrayOf())?.let { usedTargets = it.toList() }

      table?.get("cameraTargets/indices", 0)?.let { indices = it }

      table?.get("distCoeff", MatBuilder.fill(Nat.N5(), Nat.N1(), *DoubleArray(5) { 0.0 }).data)
        ?.let { distCoeff = MatBuilder.fill(Nat.N5(), Nat.N1(), *it) }

      table?.get("cameraMatrix", MatBuilder.fill(Nat.N3(), Nat.N3(), *DoubleArray(9) { 0.0 }).data)
        ?.let { cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), *it) }

      cameraTargets = mutableListOf<PhotonTrackedTarget>()

      for (targetID in 0 until indices) {
        table?.get("cameraTargets/$targetID", ByteArray(photonPacketSerde.maxByteSize))?.let {
          val photonPacket = Packet(it)
          cameraTargets.add(photonPacketSerde.unpack(photonPacket))
        }
      }
    }
  }

  fun updateInputs(inputs: CameraInputs) {}
}
