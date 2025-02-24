package com.team4099.robot2025.util

import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.lib.trajectory.CustomTrajectoryGenerator
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrajectoryParameterizer
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint
import edu.wpi.first.wpilibj.DriverStation

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisAccels
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond

class CustomTrajectory(
    val drivetrain: Drivetrain,
    val poseSupplier: () -> Pose2d,
    val trajectory: TrajectoryTypes,
    val trajectoryGenerator: CustomTrajectoryGenerator,
    val swerveDriveController: CustomHolonomicDriveController,
    val stateFrame: FrameType
) {
    val totalStates: Int
        get() {
            return when (trajectory) {
                is TrajectoryTypes.WPILib -> trajectory.rawTrajectory.states.size
                is TrajectoryTypes.Choreo -> trajectory.rawTrajectory.poses.size
            }
        }

    val timeAtFirstState: Time
        get() {
            return when (trajectory) {
                is TrajectoryTypes.WPILib -> trajectory.rawTrajectory.states[0].timeSeconds.seconds
                is TrajectoryTypes.Choreo -> trajectory.rawTrajectory.getInitialSample(false).get().timestamp.seconds
            }
        }

    val totalTime: Time
        get() {
            return when (trajectory) {
                is TrajectoryTypes.WPILib -> trajectory.rawTrajectory.totalTimeSeconds.seconds
                is TrajectoryTypes.Choreo -> trajectory.rawTrajectory.totalTime.seconds
            }
        }

    fun sample(time: Time) : Pair<Request.DrivetrainRequest.ClosedLoop, Pose2d> {
        return when (trajectory) {
            is TrajectoryTypes.WPILib -> {
                val desiredState = trajectory.rawTrajectory.sample(time.inSeconds)
                val desiredRotation = trajectoryGenerator.holonomicRotationSequence.sample(time.inSeconds)
                val poseReference = if (stateFrame == FrameType.ODOMETRY) {
                    drivetrain.odomTField.inverse().asPose2d().transformBy(poseSupplier().asTransform2d())
                } else {
                    poseSupplier()
                }.pose2d

                val nextDriveState = swerveDriveController.calculate(
                    poseReference,
                    AllianceFlipUtil.apply(desiredState),
                    AllianceFlipUtil.apply(desiredRotation)
                )

                val chassisSpeeds = ChassisSpeeds(nextDriveState.vxMetersPerSecond, nextDriveState.vyMetersPerSecond, nextDriveState.omegaRadiansPerSecond)

                val xAccel =
                    desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
                            desiredState.curvatureRadPerMeter.radians.cos
                val yAccel =
                    desiredState.accelerationMetersPerSecondSq.meters.perSecond.perSecond *
                            desiredState.curvatureRadPerMeter.radians.sin
                val chassisAccels = ChassisAccels(xAccel, yAccel, 0.0.radians.perSecond.perSecond).chassisAccelsWPILIB

                // Retrieve the target pose
                val targetPose =
                    AllianceFlipUtil.apply(
                        Pose2d(
                            desiredState.poseMeters.x.meters,
                            desiredState.poseMeters.y.meters,
                            desiredRotation.position.radians.radians
                        )
                    )

                Request.DrivetrainRequest.ClosedLoop(chassisSpeeds, chassisAccels) to targetPose
            }
            is TrajectoryTypes.Choreo -> {
                val desiredState = trajectory.rawTrajectory.sampleAt(time.inSeconds, AllianceFlipUtil.shouldFlip()).get()

                val poseReference =
                    if (stateFrame == FrameType.ODOMETRY) {
                        drivetrain.odomTField.inverse().asPose2d().transformBy(poseSupplier().asTransform2d())
                    } else {
                        poseSupplier()
                    }.pose2d

                val nextDriveState = swerveDriveController.calculate(
                    poseReference,
                    desiredState
                )

                // Retrieve the target pose
                val targetPose = Pose2d(desiredState.pose)

                Request.DrivetrainRequest.ClosedLoop(nextDriveState) to targetPose
            }
        }
    }

    companion object {
        fun fromWaypoints(
            drivetrain: Drivetrain,
            waypoints: () -> List<Waypoint>,
            constraints: List<TrajectoryConstraint> = listOf(),
            endVelocity: Velocity2d = Velocity2d()
        ) : CustomTrajectoryGenerator {
            val trajectoryGenerator = CustomTrajectoryGenerator()
            val config =
                edu.wpi.first.math.trajectory.TrajectoryConfig(
                    DrivetrainConstants.MAX_AUTO_VEL.inMetersPerSecond,
                    DrivetrainConstants.MAX_AUTO_ACCEL.inMetersPerSecondPerSecond
                )
                    .setKinematics(
                        SwerveDriveKinematics(
                            *(drivetrain.moduleTranslations.map { it.translation2d }).toTypedArray()
                        )
                    )
                    .setStartVelocity(drivetrain.fieldVelocity.magnitude.inMetersPerSecond)
                    .setEndVelocity(endVelocity.magnitude.inMetersPerSecond)
                    .addConstraint(
                        CentripetalAccelerationConstraint(
                            DrivetrainConstants.STEERING_ACCEL_MAX.inRadiansPerSecondPerSecond
                        )
                    )
                    .addConstraints(constraints)

            try {
                trajectoryGenerator.generate(config, waypoints())
            } catch (exception: TrajectoryParameterizer.TrajectoryGenerationException) {
                DriverStation.reportError("Failed to generate trajectory.", true)
            }

            return trajectoryGenerator
        }
    }
}