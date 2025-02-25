package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.lib.trajectory.CustomTrajectoryGenerator
import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.lib.trajectory.OdometryWaypoint
import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.CustomTrajectory
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.FrameType
import com.team4099.robot2025.util.TrajectoryTypes
import com.team4099.robot2025.util.Velocity2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.hal.Clock
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadian
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianSeconds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond
import java.util.function.Supplier
import kotlin.math.PI
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class DrivePathCommand<T : Waypoint>
private constructor(
    val drivetrain: Drivetrain,
    private val trajectory: TrajectoryTypes,
    val resetPose: Boolean = false,
    val useLowerTolerance: Boolean = false,
    val flipForAlliances: Boolean = true,
    val endPathOnceAtReference: Boolean = true,
    val leaveOutYAdjustment: Boolean = false,
    val keepTrapping: Boolean = true,
    val endVelocity: Velocity2d = Velocity2d(),
    var stateFrame: FrameType = FrameType.ODOMETRY,
    var pathFrame: FrameType = FrameType.FIELD,
    val trajectoryGenerator: CustomTrajectoryGenerator = CustomTrajectoryGenerator()
) : Command() {
    private val xPID: PIDController<Meter, Velocity<Meter>>
    private val yPID: PIDController<Meter, Velocity<Meter>>

    private val thetaPID: PIDController<Radian, Velocity<Radian>>

    private val swerveDriveController: CustomHolonomicDriveController

    private var trajCurTime = 0.0.seconds
    private var trajStartTime = 0.0.seconds

    private var generatedTrajectory: CustomTrajectory

    private var changeStartTimeOnExecute = true

    val thetakP =
        LoggedTunableValue(
            "Pathfollow/thetakP",
            Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian })
        )
    val thetakI =
        LoggedTunableValue(
            "Pathfollow/thetakI",
            Pair(
                { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }
            )
        )
    val thetakD =
        LoggedTunableValue(
            "Pathfollow/thetakD",
            Pair(
                { it.inRadiansPerSecondPerRadianPerSecond },
                { it.radians.perSecond.perRadianPerSecond }
            )
        )

    val thetaMaxVel =
        LoggedTunableValue("Pathfollow/thetaMaxVel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_VEL)
    val thetaMaxAccel =
        LoggedTunableValue("Pathfollow/thetaMaxAccel", DrivetrainConstants.PID.MAX_AUTO_ANGULAR_ACCEL)

    val poskP =
        LoggedTunableValue(
            "Pathfollow/posKP",
            DrivetrainConstants.PID.AUTO_POS_KP,
            Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
        )
    val poskI =
        LoggedTunableValue(
            "Pathfollow/posKI",
            DrivetrainConstants.PID.AUTO_POS_KI,
            Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
        )
    val poskD =
        LoggedTunableValue(
            "Pathfollow/posKD",
            DrivetrainConstants.PID.AUTO_POS_KD,
            Pair(
                { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
            )
        )

    private var lastSampledPose = Pose2d()
    private lateinit var pathTransform: Transform2d

    private var drivePoseSupplier: () -> Pose2d
    private var odoTField: Transform2d = Transform2d(Translation2d(), 0.0.degrees)

    private var errorString = ""

    init {
        addRequirements(drivetrain)
        if (RobotBase.isReal()) {
            thetakP.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KD)
        } else {
            thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
        }

        xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
        yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
        thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())

        thetaPID.enableContinuousInput(-PI.radians, PI.radians)

        when (stateFrame) {
            FrameType.ODOMETRY -> drivePoseSupplier = { drivetrain.odomTRobot }
            FrameType.FIELD -> {
                drivePoseSupplier = { drivetrain.fieldTRobot }
                // if we're already in field frame we do not want to shift by `odoTField` again
                pathFrame = FrameType.FIELD
            }
        }

        swerveDriveController =
            CustomHolonomicDriveController(
                xPID.wpiPidController, yPID.wpiPidController, thetaPID.wpiPidController
            )

        if (useLowerTolerance) {
            swerveDriveController.setTolerance(Pose2d(3.inches, 3.inches, 1.5.degrees).pose2d)
        } else {
            swerveDriveController.setTolerance(Pose2d(6.inches, 6.inches, 6.degrees).pose2d)
        }

        // drop this all into the custom trajectory
        generatedTrajectory = CustomTrajectory(
            drivetrain,
            drivePoseSupplier,
            trajectory,
            trajectoryGenerator,
            swerveDriveController,
            stateFrame
        )
    }

    override fun initialize() {
        if (generatedTrajectory.totalStates <= 1) {
            return
        }

        odoTField = drivetrain.odomTField

        //    if (resetPose) {
        //      drivetrain.odometryPose = AllianceFlipUtil.apply(Pose2d(trajectory.initialPose))
        //    }
        trajStartTime = Clock.fpgaTime + generatedTrajectory.timeAtFirstState
        thetaPID.reset()
        xPID.reset()
        yPID.reset()
    }

    override fun execute() {
        if (generatedTrajectory.totalStates <= 1) {
            return
        }

        if (changeStartTimeOnExecute) {
            trajStartTime = Clock.fpgaTime + generatedTrajectory.timeAtFirstState
            changeStartTimeOnExecute = false
        }

        trajCurTime = Clock.fpgaTime - trajStartTime
        val (nextDriveRequest, targetPose) = generatedTrajectory.sample(trajCurTime)

        var robotPoseInSelectedFrame: Pose2d = drivePoseSupplier()
        if (pathFrame == stateFrame) {
            lastSampledPose = targetPose
            when (stateFrame) {
                FrameType.FIELD -> {
                    CustomLogger.recordOutput("Pathfollow/fieldTRobotTargetVisualized", targetPose.pose2d)
                    CustomLogger.recordOutput("Pathfollow/fieldTRobot", robotPoseInSelectedFrame.pose2d)
                }

                FrameType.ODOMETRY -> {
                    CustomLogger.recordOutput("Pathfollow/odomTRobotTargetVisualized", targetPose.pose2d)
                    CustomLogger.recordOutput("Pathfollow/odomTRobot", robotPoseInSelectedFrame.pose2d)
                }
            }

            //        pathTransform.inverse().asPose2d().transformBy(targetHolonomicPose.asTransform2d())
        } else {
            when (pathFrame) {
                FrameType.ODOMETRY -> {
                    // TODO (saraansh) we disallow this, not possible to get to. remove or find use case
                    lastSampledPose =
                        odoTField.inverse().asPose2d().transformBy(targetPose.asTransform2d())
                }

                FrameType.FIELD -> {
                    // robotPose is currently odomTrobot we want fieldTRobot. we obtain that via fieldTodo x
                    // odoTRobot
                    robotPoseInSelectedFrame =
                        odoTField.inverse().asPose2d().transformBy(robotPoseInSelectedFrame.asTransform2d())
                    lastSampledPose = odoTField.asPose2d().transformBy(targetPose.asTransform2d())

                    CustomLogger.recordOutput("Pathfollow/fieldTRobotTargetVisualized", targetPose.pose2d)
                    CustomLogger.recordOutput("Pathfollow/fieldTRobot", robotPoseInSelectedFrame.pose2d)
                }
            }
        }

        /*
        drivetrain.setOpenLoop(
            nextDriveState.omegaRadiansPerSecond.radians.perSecond,
            nextDriveState.vxMetersPerSecond.meters.perSecond to nextDriveState.vyMetersPerSecond.meters.perSecond,
            ChassisAccels(xAccel, yAccel, 0.0.radians.perSecond.perSecond).chassisAccelsWPILIB,
            true
          )

         */

        drivetrain.currentRequest =
            nextDriveRequest

        CustomLogger.recordDebugOutput("Pathfollow/thetaPIDPositionErrorRadians", thetaPID.error.inRadians)

        CustomLogger.recordDebugOutput("Pathfollow/xPIDPositionErrorMeters", xPID.error.inMeters)
        CustomLogger.recordDebugOutput("Pathfollow/yPIDPositionErrorMeters", yPID.error.inMeters)
        CustomLogger.recordDebugOutput(
            "Pathfollow/thetaPIDVelocityErrorRadians", thetaPID.errorDerivative.inRadiansPerSecond
        )

        CustomLogger.recordDebugOutput(
            "Pathfollow/xAccelMetersPerSecondPerSecond", nextDriveRequest.chassisAccels.vxMetersPerSecond
        )
        CustomLogger.recordDebugOutput(
            "Pathfollow/yAccelMetersPerSecondPerSecond", nextDriveRequest.chassisAccels.vyMetersPerSecond
        )

        CustomLogger.recordDebugOutput("Pathfollow/Start Time", trajStartTime.inSeconds)
        CustomLogger.recordDebugOutput("Pathfollow/Current Time", trajCurTime.inSeconds)
        CustomLogger.recordDebugOutput(
            "Pathfollow/Desired Angle in Degrees", targetPose.pose2d.rotation.degrees
        )

        CustomLogger.recordDebugOutput("Pathfollow/isAtReference", swerveDriveController.atReference())
        CustomLogger.recordDebugOutput("Pathfollow/trajectoryTimeSeconds", generatedTrajectory.totalTime.inSeconds)

        CustomLogger.recordDebugOutput("ActiveCommands/DrivePathCommand", true)

        if (thetakP.hasChanged()) thetaPID.proportionalGain = thetakP.get()
        if (thetakI.hasChanged()) thetaPID.integralGain = thetakI.get()
        if (thetakD.hasChanged()) thetaPID.derivativeGain = thetakD.get()

        if (poskP.hasChanged()) {
            xPID.proportionalGain = poskP.get()
            yPID.proportionalGain = poskP.get()
        }
        if (poskI.hasChanged()) {
            xPID.integralGain = poskI.get()
            yPID.integralGain = poskI.get()
        }
        if (poskD.hasChanged() && poskD.hasChanged()) {
            xPID.derivativeGain = poskD.get()
            yPID.derivativeGain = poskD.get()
        }
    }

    override fun isFinished(): Boolean {
        trajCurTime = Clock.fpgaTime - trajStartTime

        if (!keepTrapping) {
            return endPathOnceAtReference && trajCurTime > generatedTrajectory.totalTime
        }

        return endPathOnceAtReference &&
        (
                swerveDriveController.atReference()
                    && (drivePoseSupplier().rotation - generatedTrajectory.endPose.rotation).absoluteValue < 5.degrees
                    && trajCurTime > generatedTrajectory.totalTime
            )
    }

    override fun end(interrupted: Boolean) {
        CustomLogger.recordDebugOutput("ActiveCommands/DrivePathCommand", false)
        if (interrupted) {
            DriverStation.reportError(errorString, true)
            // Stop where we are if interrupted
            drivetrain.currentRequest =
                DrivetrainRequest.OpenLoop(
                    0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
                )
        } else {
            // Execute one last time to end up in the final state of the trajectory
            // Since we weren't interrupted, we know curTime > endTime
            execute()
            drivetrain.currentRequest =
                DrivetrainRequest.OpenLoop(
                    0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
                )
        }
    }

    companion object {
        fun createPathInOdometryFrame(
            drivetrain: Drivetrain,
            trajectory: TrajectoryTypes,
            resetPose: Boolean = false,
            useLowerTolerance: Boolean = false,
            flipForAlliances: Boolean = true,
            endPathOnceAtReference: Boolean = true,
            leaveOutYAdjustment: Boolean = false,
            keepTrapping: Boolean = false,
            endVelocity: Velocity2d = Velocity2d(),
            stateFrame: FrameType = FrameType.ODOMETRY,
        ): DrivePathCommand<OdometryWaypoint> =
            DrivePathCommand(
                drivetrain,
                trajectory,
                resetPose,
                useLowerTolerance,
                flipForAlliances,
                endPathOnceAtReference,
                leaveOutYAdjustment,
                keepTrapping,
                endVelocity,
                stateFrame,
                FrameType.ODOMETRY
            )

        fun createPathInOdometryFrame(
            drivetrain: Drivetrain,
            trajectoryGenerator: CustomTrajectoryGenerator,
            resetPose: Boolean = false,
            useLowerTolerance: Boolean = false,
            flipForAlliances: Boolean = true,
            endPathOnceAtReference: Boolean = true,
            leaveOutYAdjustment: Boolean = false,
            keepTrapping: Boolean = false,
            endVelocity: Velocity2d = Velocity2d(),
            stateFrame: FrameType = FrameType.ODOMETRY,
        ): DrivePathCommand<FieldWaypoint> =
            DrivePathCommand(
                drivetrain,
                TrajectoryTypes.WPILib(trajectoryGenerator.driveTrajectory),
                resetPose,
                useLowerTolerance,
                flipForAlliances,
                endPathOnceAtReference,
                leaveOutYAdjustment,
                keepTrapping,
                endVelocity,
                stateFrame,
                FrameType.ODOMETRY,
                trajectoryGenerator
            )

        fun createPathInFieldFrame(
            drivetrain: Drivetrain,
            trajectory: TrajectoryTypes,
            resetPose: Boolean = false,
            useLowerTolerance: Boolean = false,
            flipForAlliances: Boolean = true,
            endPathOnceAtReference: Boolean = true,
            leaveOutYAdjustment: Boolean = false,
            keepTrapping: Boolean = true,
            endVelocity: Velocity2d = Velocity2d(),
            stateFrame: FrameType = FrameType.ODOMETRY,
        ): DrivePathCommand<FieldWaypoint> =
            DrivePathCommand(
                drivetrain,
                trajectory,
                resetPose,
                useLowerTolerance,
                flipForAlliances,
                endPathOnceAtReference,
                leaveOutYAdjustment,
                keepTrapping,
                endVelocity,
                stateFrame,
                FrameType.FIELD
            )

        fun createPathInFieldFrame(
            drivetrain: Drivetrain,
            trajectoryGenerator: CustomTrajectoryGenerator,
            resetPose: Boolean = false,
            useLowerTolerance: Boolean = false,
            flipForAlliances: Boolean = true,
            endPathOnceAtReference: Boolean = true,
            keepTrapping: Boolean = true,
            leaveOutYAdjustment: Boolean = false,
            endVelocity: Velocity2d = Velocity2d(),
            stateFrame: FrameType = FrameType.ODOMETRY,
        ): DrivePathCommand<FieldWaypoint> =
            DrivePathCommand(
                drivetrain,
                TrajectoryTypes.WPILib(trajectoryGenerator.driveTrajectory),
                resetPose,
                useLowerTolerance,
                flipForAlliances,
                endPathOnceAtReference,
                leaveOutYAdjustment,
                keepTrapping,
                endVelocity,
                stateFrame,
                FrameType.FIELD,
                trajectoryGenerator
            )
    }
}