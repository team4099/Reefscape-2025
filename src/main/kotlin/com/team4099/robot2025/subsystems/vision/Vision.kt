package com.team4099.robot2023.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.logging.toDoubleArray
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2025.subsystems.vision.camera.CameraIO
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.util.FMSData
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonUtils
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import java.util.function.Consumer
import java.util.function.Supplier

class Vision(vararg cameras: CameraIO) : SubsystemBase() {
    val io: List<CameraIO> = cameras.toList()
    val inputs = List(io.size) { CameraIO.CameraInputs() }

    var drivetrainOdometry: () -> Pose2d = { Pose2d() }
    var robotTSpeaker: Translation3d = Translation3d()
    var trustedRobotDistanceToTarget: Length = 0.meters

    companion object {
        val ambiguityThreshold = 0.7
        val targetLogTime = 0.05.seconds

        val xyStdDevCoeffecient = 0.05
        val thetaStdDevCoefficient = 1.5
    }

    private val xyStdDevCoefficient = TunableNumber("Vision/xystdev", xyStdDevCoeffecient)

    private val thetaStdDev = TunableNumber("Vision/thetaStdDev", thetaStdDevCoefficient)

    private var fieldFramePoseSupplier = Supplier<Pose2d> { Pose2d() }
    private var visionConsumer: Consumer<List<TimestampedVisionUpdate>> = Consumer {}
    private var speakerVisionConsumer: Consumer<TimestampedTrigVisionUpdate> = Consumer {}
    private val lastFrameTimes = mutableMapOf<Int, Time>()
    private var lastDetectionTime = 0.0.seconds

    init {
        for (i in io.indices) {
            lastFrameTimes[i] = 0.0.seconds
        }
    }

    fun setDataInterfaces(
        fieldFramePoseSupplier: Supplier<Pose2d>,
        visionConsumer: Consumer<List<TimestampedVisionUpdate>>,
        speakerVisionMeasurementConsumer: Consumer<TimestampedTrigVisionUpdate>
    ) {
        this.fieldFramePoseSupplier = fieldFramePoseSupplier
        this.visionConsumer = visionConsumer
        this.speakerVisionConsumer = speakerVisionMeasurementConsumer
    }

    override fun periodic() {
        val startTime = Clock.realTimestamp

        for (instance in io.indices) {
            io[instance].updateInputs(inputs[instance])
            //Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
        }

        for (instance in io.indices) {


        }

        Logger.recordOutput(
            "LoggedRobot/VisionLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds
        )
    }
}