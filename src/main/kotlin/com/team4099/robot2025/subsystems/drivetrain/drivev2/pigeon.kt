package com.team4099.robot2025.subsystems.drivetrain.drivev2

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO.GyroIOInputs
import com.team4099.utils.threads.PhoenixOdometryThread
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.generated.TunerConstants
import java.util.*
import java.util.function.IntFunction

/** IO implementation for Pigeon 2.  */
class GyroIOPigeon2 : GyroIO {
    private val pigeon: Pigeon2 = Pigeon2(
        TunerConstants.DrivetrainConstants.Pigeon2Id,
        TunerConstants.DrivetrainConstants.CANBusName
    )
    private val yaw: StatusSignal<Angle> = pigeon.yaw
    private val yawPositionQueue: Queue<Double>
    private val yawTimestampQueue: Queue<Double>
    private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
        yawVelocity.setUpdateFrequency(50.0)
        pigeon.optimizeBusUtilization()
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone())
    }

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble { value: Double? -> value!! }.toArray()
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map<Rotation2d> { value: Double? ->
                    Rotation2d.fromDegrees(
                        value!!
                    )
                }
                .toArray<Rotation2d>(IntFunction<Array<Rotation2d>> { _Dummy_.__Array__() })
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}