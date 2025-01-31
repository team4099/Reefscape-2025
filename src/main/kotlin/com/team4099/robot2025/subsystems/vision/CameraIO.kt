package com.team4099.robot2025.subsystems.vision.camera

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.common.dataflow.structures.Packet
import org.photonvision.struct.PhotonPipelineResultSerde
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Pose3dWPILIB
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

interface CameraIO {
    class CameraInputs : LoggableInputs {
        var timestamp = 0.0.seconds
        var cameraTargets = mutableListOf<PhotonPipelineResult>()
        var distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *DoubleArray(5) { 0.0 })
        var indices = 0

        var photonPipelineResultSerde = PhotonPipelineResultSerde()

        override fun toLog(table: LogTable?) {
            table?.put("timestampSeconds", timestamp.inSeconds)
            table?.put("distCoeff", distCoeff.data)
            table?.put("pipelineSize", cameraTargets.size)

            for (targetID in cameraTargets.indices) {
                val photonPacket = Packet(photonPipelineResultSerde.maxByteSize)
                photonPipelineResultSerde.pack(photonPacket, cameraTargets.get(targetID))
                table?.put("cameraTargets/$targetID", photonPacket.writtenDataCopy)
            }

            table?.put("cameraTargets/indices", cameraTargets.size)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("timestampSeconds", 0.0)?.let { timestamp = it.seconds }
            table?.get("fps", 0.0)
            table?.get("distCoeff", MatBuilder.fill(Nat.N8(), Nat.N1(), *DoubleArray(8) { 0.0 }).data)
                ?.let { distCoeff = MatBuilder.fill(Nat.N8(), Nat.N1(), *it) }
            table?.get("pipelineSize", 0)?.let { indices = it}

            cameraTargets = mutableListOf<PhotonPipelineResult>()

            for (targetID in 0 until indices) {
                table?.get("cameraTargets/$targetID", ByteArray(photonPipelineResultSerde.maxByteSize))?.let {
                    val photonPacket = Packet(it)
                    cameraTargets.add(photonPipelineResultSerde.unpack(photonPacket))
                }
            }
        }
    }

    fun updateInputs(inputs: CameraInputs) {}
}