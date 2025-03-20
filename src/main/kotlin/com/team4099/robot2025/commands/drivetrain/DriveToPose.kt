package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.Velocity2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.interpolate
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt


class DriveToPose(
    val drivetrain: Drivetrain,
    val targetPoseSupplier: () -> Pose2d,
    val robotPoseSupplier: () -> Pose2d,
    val linearFF: () -> Pair<Double, Double> = {Pair(0.0, 0.0)},
    val omegaFF: () -> Double = {0.0}
    ) : Command() {


    var driveController: ProfiledPIDController<Meter, Velocity<Meter>>
    val driveConstraints = TrapezoidProfile.Constraints(DrivetrainConstants.MAX_REEF_VEL, DrivetrainConstants.MAX_REEF_ACCEL)

    var thetaController: ProfiledPIDController<Radian, Velocity<Radian>>
    val thetaConstraints = TrapezoidProfile.Constraints(DrivetrainConstants.MAX_REEF_ANGULAR_VEL, DrivetrainConstants.MAX_REEF_ANGULAR_ACCEL)

    var currentPose = Pose2d()
    var targetPose = Pose2d()

    var translationToTarget = Translation2d()

    var setpointPose = Pose2d()
    var positionError = 0.0.meters
    var thetaError = 0.0.degrees

    val drivekP =
        LoggedTunableValue(
            "DriveToPose/drivekP", Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
        )
    val drivekI =
        LoggedTunableValue(
            "DriveToPose/drivekI",
            Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
        )
    val drivekD =
        LoggedTunableValue(
            "DriveToPose/drivekD",
            Pair(
                { it.inMetersPerSecondPerMeterPerSecond }, { it.meters.perSecond.perMeterPerSecond }
            )
        )

        val thetakP =
            LoggedTunableValue(
                "DriveToPose/thetakP",
                Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
            )
        val thetakI =
            LoggedTunableValue(
                "DriveToPose/thetakI",
                Pair(
                    { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
                )
            )
        val thetakD =
            LoggedTunableValue(
                "DriveToPose/thetakD",
                Pair(
                    { it.inDegreesPerSecondPerDegreePerSecond },
                    { it.degrees.perSecond.perDegreePerSecond }
                )
            )

    init {

        addRequirements(drivetrain)

        if (RobotBase.isReal()) {

            drivekP.initDefault(DrivetrainConstants.PID.REEF_POS_PID_KP)
            drivekI.initDefault(DrivetrainConstants.PID.REEF_POS_PID_KI)
            drivekD.initDefault(DrivetrainConstants.PID.REEF_POS_PID_KD)

            thetakP.initDefault(DrivetrainConstants.PID.REEF_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.REEF_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.REEF_THETA_PID_KD)

        } else {

            drivekP.initDefault(DrivetrainConstants.PID.SIM_REEF_POS_PID_KP)
            drivekI.initDefault(DrivetrainConstants.PID.SIM_REEF_POS_PID_KI)
            drivekD.initDefault(DrivetrainConstants.PID.SIM_REEF_POS_PID_KD)

            thetakP.initDefault(DrivetrainConstants.PID.SIM_REEF_THETA_PID_KP)
            thetakI.initDefault(DrivetrainConstants.PID.SIM_REEF_THETA_PID_KI)
            thetakD.initDefault(DrivetrainConstants.PID.SIM_REEF_THETA_PID_KD)
        }

        driveController =
            ProfiledPIDController(
                drivekP.get(),
                drivekI.get(),
                drivekD.get(),
                driveConstraints
            )

        thetaController =
            ProfiledPIDController(
                thetakP.get(),
                thetakI.get(),
                thetakD.get(),
                thetaConstraints
            )

        thetaController.enableContinuousInput(-PI.radians, PI.radians)
    }

    override fun initialize() {
        if (drivekP.hasChanged() || drivekI.hasChanged() || drivekD.hasChanged()) {
            driveController =
                ProfiledPIDController(
                    drivekP.get(),
                    drivekI.get(),
                    drivekD.get(),
                    driveConstraints
                )
        }

        if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
            thetaController =
                ProfiledPIDController(
                    thetakP.get(),
                    thetakI.get(),
                    thetakD.get(),
                    thetaConstraints
                )

            thetaController.enableContinuousInput(-PI.radians, PI.radians)
        }


        currentPose = robotPoseSupplier()
        setpointPose = currentPose
        val fieldVelocity = drivetrain.fieldVelocity
        translationToTarget = targetPoseSupplier().translation - currentPose.translation

        //positive velocity is towards target, negative is away
        var velocityTowardsTarget = fieldVelocity.rotateBy(-translationToTarget.translation2d.angle.radians.radians).x
        if (velocityTowardsTarget < 0.0.meters.perSecond) velocityTowardsTarget = 0.0.meters.perSecond
        Logger.recordOutput("DriveToPose/initialVelocityTowardsTarget", velocityTowardsTarget.inMetersPerSecond)

        //flip velocityTowards target since distance is decreasing when velocity is positive
        driveController.reset(
            TrapezoidProfile.State(
            translationToTarget.magnitude.meters , -velocityTowardsTarget
            )
        )
        driveController.setGoal(0.0.meters)

        thetaController.reset(
            TrapezoidProfile.State(
                currentPose.rotation, drivetrain.omegaVelocity
            )
        )
        thetaController.setGoal(targetPose.rotation)


    }

    override fun execute() {

        CustomLogger.recordDebugOutput("ActiveCommands/DriveToPose", true)

        currentPose = robotPoseSupplier()
        targetPose = targetPoseSupplier()


        //updating trapezoidal profile based on new target
        driveController.reset(
            TrapezoidProfile.State
                (
                (setpointPose.translation - targetPose.translation).magnitude.meters,
                driveController.setpoint.velocity
                        )
        )

        //calculating error from goal
        translationToTarget = targetPoseSupplier().translation - currentPose.translation
        positionError = translationToTarget.magnitude.meters
        thetaError = (currentPose.rotation - targetPose.rotation)

        //calculate drive velocity output towards target using trapezoidal velocity feedforward and controller feedback
        val driveVelocityTarget =
            driveController.setpoint.velocity + driveController.calculate(positionError, 0.0.meters)


        //calculate theta velocity output using trapezoidal velocity feedforward and controller feedback
        Logger.recordOutput("DriveToPose/currentPoseRotation", currentPose.rotation.inDegrees)
        var thetaVelocityTarget: AngularVelocity = thetaController.setpoint.velocity + thetaController.calculate(currentPose.rotation, targetPose.rotation)


        val setpointTranslation = Pose2d(
            targetPose.translation,
            atan2(
                (currentPose.translation.y - targetPose.translation.y).inMeters,
                (currentPose.translation.x - targetPose.translation.x).inMeters
            ).radians
        ).transformBy(
            Transform2d(
                Translation2d(driveController.setpoint.position, 0.0.meters),
                0.0.radians
            )
        ).translation

        setpointPose = Pose2d(setpointTranslation, thetaController.setpoint.position)

        var fieldVelocityTarget = Velocity2d.fromVelocityVectorToVelocity2d(
            driveVelocityTarget,
            atan2(
                (currentPose.translation.y - targetPose.translation.y).inMeters,
                (currentPose.translation.x - targetPose.translation.x).inMeters
            ).radians
        )

        //Balance driver override control using arbitrary feedforward
        val linearS = (sqrt(linearFF().first.pow(2) + linearFF().second.pow(2)) * 3).seconds
        val thetaS = (omegaFF() * 3.0).seconds


        fieldVelocityTarget = fieldVelocityTarget.interpolate(
            Velocity2d(
                DrivetrainConstants.MAX_REEF_VEL * linearFF().first,
                DrivetrainConstants.MAX_REEF_VEL * linearFF().second
            ),
            linearS
        )

        thetaVelocityTarget = interpolate(
            thetaVelocityTarget,
            DrivetrainConstants.MAX_REEF_ANGULAR_VEL * omegaFF(),
            thetaS.inSeconds
        )



        drivetrain.currentRequest = Request.DrivetrainRequest.ClosedLoop(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldVelocityTarget.x.inMetersPerSecond,
                fieldVelocityTarget.y.inMetersPerSecond,
                thetaVelocityTarget.inRadiansPerSecond,
                currentPose.rotation.inRotation2ds)
        )

        Logger.recordOutput("DriveToPose/currentPose", currentPose.pose2d)
        Logger.recordOutput("DriveToPose/setpointPose", setpointPose.pose2d)
        Logger.recordOutput("DriveToPose/targetPose", targetPose.pose2d)


        Logger.recordOutput("DriveToPose/profiledDriveDistanceMeters", driveController.setpoint.position.inMeters)
        Logger.recordOutput("DriveToPose/profiledDriveVelocityMetersPerSecond", driveController.setpoint.velocity.inMetersPerSecond)

        Logger.recordOutput("DriveToPose/positionErrorFromTargetMeters", positionError.inMeters)
        Logger.recordOutput("DriveToPose/driveVelocityTargetMetersPerSecond", driveVelocityTarget.inMetersPerSecond)

        Logger.recordOutput("DriveToPose/fieldVelocityTarget", fieldVelocityTarget.velocity2dWPIlib)

        Logger.recordOutput("DriveToPose/profiledThetaAngleDegrees", thetaController.setpoint.position.inDegrees)
        Logger.recordOutput("DriveToPose/profiledThetaVelocityDegreesPerSecond", thetaController.setpoint.velocity.inDegreesPerSecond)

        Logger.recordOutput("DriveToPose/thetaErrorFromTargetDegrees", thetaError.inDegrees)
        Logger.recordOutput("DriveToPose/thetaVelocityTargetDegreesPerSecond", thetaVelocityTarget.inDegreesPerSecond)

    }

    override fun end(interupted: Boolean) {

        CustomLogger.recordDebugOutput("ActiveCommands/DriveToPose", false)

        drivetrain.currentRequest =
            DrivetrainRequest.OpenLoop(
                0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
            )

        Logger.recordOutput("DriveToPose/currentPose", Pose2d().pose2d)
        Logger.recordOutput("DriveToPose/setpointPose", Pose2d().pose2d)
        Logger.recordOutput("DriveToPose/targetPose", Pose2d().pose2d)
    }

    fun atTarget(): Boolean {
        return driveController.atGoal && thetaController.atGoal
    }

    fun withinTolerance(driveTolerance: Length, thetaTolerance: Angle): Boolean {
        return positionError.absoluteValue < driveTolerance && thetaError.absoluteValue < thetaTolerance
    }

}