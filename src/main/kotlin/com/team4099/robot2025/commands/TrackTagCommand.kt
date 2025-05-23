package com.team4099.robot2025.commands

import com.team4099.robot2025.commands.drivetrain.TargetAngleCommand
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.rollers.Ramp
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts


class TrackTagCommand(
    val driver: DriverProfile,
    val driveX: () -> Double,
    val driveY: () -> Double,
    val turn: () -> Double,
    val slowMode: () -> Boolean,
    val drivetrain: Drivetrain,
    val vision: Vision,
    val ramp: Ramp
) : Command() {
    private var robotAngleFromTag = 0.0

    private val aimCommand: TargetAngleCommand
        get() = TargetAngleCommand(
            driver,
            driveX,
            driveY,
            turn,
            slowMode,
            drivetrain,
            { -0.5.radians * robotAngleFromTag.degrees.inRadians }
        )

    override fun initialize() {
        addRequirements(vision)
    }


    override fun execute() {
        val cams = vision.inputs
        var aimedTowardsTag = false

        if (cams.size >= 2 && cams[0].cameraTargets.size > 0 && cams[1].cameraTargets.size > 0) {
            val raven1 = cams[0].cameraTargets[0]
            val raven2 = cams[1].cameraTargets[0]
            // TODO: Make constant threshold
            aimedTowardsTag = abs(raven1.getYaw() + raven2.getYaw()) < 5 && raven1.fiducialId == raven2.fiducialId
        }

        CustomLogger.recordOutput("Vision/aimedTowardsTag", aimedTowardsTag)

        if (aimedTowardsTag) {
            aimCommand.end(true)
        } else {
            CustomLogger.recordDebugOutput("Vision/cameras", cams.size)

            if (cams.size >= 2) {
                if (cams[0].cameraTargets.size > 0 && cams[1].cameraTargets.size > 0) {
                    val raven1Angle = cams[0].cameraTargets[0].getYaw()
                    val raven2Angle = cams[1].cameraTargets[0].getYaw()
                    robotAngleFromTag = ((raven1Angle + raven2Angle) / 2)
                } else if (cams[0].cameraTargets.size > 0) {
                    robotAngleFromTag = -abs(cams[0].cameraTargets[0].getYaw())
                } else if (cams[1].cameraTargets.size > 0) {
                    robotAngleFromTag = abs(cams[1].cameraTargets[0].getYaw())
                }
                aimCommand.execute()
            }
        }
        CustomLogger.recordOutput("Vision/loggedRobotAngleFromTag", robotAngleFromTag)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        ramp.currentRequest = Request.RampRequest.OpenLoop(0.0.volts)
    }
}
