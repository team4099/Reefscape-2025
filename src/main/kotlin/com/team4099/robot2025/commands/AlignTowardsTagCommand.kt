package com.team4099.robot2025.commands

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.rollers.Ramp
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.abs

class AlignTowardsTagCommand(
    val driver: DriverProfile,
    val driveX: () -> Double,
    val driveY: () -> Double,
    val turn: () -> Double,
    val slowMode: () -> Boolean,
    val drivetrain: Drivetrain,
    val vision: Vision,
    val ramp: Ramp
) : Command() {
    private var thetaPID: PIDController<Radian, Velocity<Radian>>

    val kP =
        LoggedTunableValue(
            "Vision/thetakP",
            Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
        )
    val kI =
        LoggedTunableValue(
            "Vision/thetakI",
            Pair(
                { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
            )
        )
    val kD =
        LoggedTunableValue(
            "Vision/thetakD",
            Pair(
                { it.inDegreesPerSecondPerDegreePerSecond },
                { it.degrees.perSecond.perDegreePerSecond }
            )
        )

    init {
        addRequirements(drivetrain)

        thetaPID =
            PIDController(
                kP.get(),
                kI.get(),
                kD.get(),
            )

        if (!(RobotBase.isSimulation())) {
            kP.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KP)
            kI.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KI)
            kD.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KD)

            thetaPID =
                PIDController(
                    kP.get(),
                    kI.get(),
                    kD.get()
                )
        } else {
            kP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
            kI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
            kD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)

            thetaPID =
                PIDController(
                    DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
                    DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
                    DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
                )
        }

        thetaPID.enableContinuousInput(-PI.radians, PI.radians)
    }

    override fun initialize() {
        addRequirements(vision);
        thetaPID.reset()
        vision.isAligned = false;
    }

    override fun execute() {
        val cams = vision.inputs

        if (cams.size >= 2 && cams[0].cameraTargets.size > 0 && cams[1].cameraTargets.size > 0) {
            val raven1 = cams[0].cameraTargets[0]
            val raven2 = cams[1].cameraTargets[0]
            // TODO: Make constant threshold
            vision.isAligned = Math.abs(raven1.yaw - raven2.yaw) < 2 && raven1.fiducialId == raven2.fiducialId
        }

        if (vision.isAligned) {
            Request.DrivetrainRequest.OpenLoop(
                0.radians.perSecond,
                driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
                fieldOriented = true
            )
        } else {
            CustomLogger.recordDebugOutput("Vision/cameras", cams.size)

            if (cams.size >= 2) {
                val raven1 = cams[0]
                val raven2 = cams[1]
                val raven1HasTargets = raven1.cameraTargets.size > 0
                val raven2HasTargets = raven2.cameraTargets.size > 0

                if (raven1HasTargets || raven2HasTargets) {
                    if (raven1HasTargets && raven2HasTargets) {
                        val raven1Angle = raven1.cameraTargets[0].yaw
                        val raven2Angle = raven2.cameraTargets[0].yaw

                        val thetaFeedback =
                            thetaPID.calculate(raven2Angle.degrees - raven1Angle.degrees, 0.0.degrees)

                        drivetrain.currentRequest =
                            Request.DrivetrainRequest.OpenLoop(
                                thetaFeedback,
                                driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
                                fieldOriented = true
                            )
                    } else if (raven1HasTargets) {
                        val thetaFeedback =
                            thetaPID.calculate(raven1.cameraTargets[0].yaw.degrees, -30.0.degrees)

                        drivetrain.currentRequest =
                            Request.DrivetrainRequest.OpenLoop(
                                thetaFeedback,
                                driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
                                fieldOriented = true
                            )
                    } else {
                        val thetaFeedback =
                            thetaPID.calculate(raven2.cameraTargets[0].yaw.degrees, 30.0.degrees)

                        drivetrain.currentRequest =
                            Request.DrivetrainRequest.OpenLoop(
                                thetaFeedback,
                                driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
                                fieldOriented = true
                            )
                    }
                } else {
                    drivetrain.currentRequest =
                        Request.DrivetrainRequest.OpenLoop(
                            0.0.radians.perSecond,
                            driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
                            fieldOriented = true
                        )
                }
            }
        }

        // CustomLogger.recordOutput("Vision/loggedRobotAngleFromTag", robotAngleFromTag)


//        val lastUpdate = vision.lastTrigVisionUpdate
//        val angleOffset = (180.degrees - lastUpdate.robotTReefTag.rotation.absoluteValue) - lastUpdate.robotTReefTag.rotation
//        val thetaFeedback = thetaPID.calculate(lastUpdate.robotTReefTag.rotation.absoluteValue, 1.0.radians)
//
//        if (!vision.isAligned && thetaPID.error.absoluteValue > 5.degrees && lastUpdate.targetTagID != -1) {
//            drivetrain.currentRequest =
//                Request.DrivetrainRequest.OpenLoop(
//                    thetaFeedback,
//                    driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
//                    fieldOriented = true
//                )
//        } else {
//            vision.isAligned = true
//
//            drivetrain.currentRequest =
//                Request.DrivetrainRequest.OpenLoop(
//                    0.0.radians.perSecond,
//                    driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
//                    fieldOriented = true
//                )
//        }
    }
}