package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.util.driver.DriverProfile
import com.team4099.robot2023.util.inverse
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2

class TargetSpeakerCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val superstructure: Superstructure,
  val vision: Vision
) : Command() {
  private var thetaPID: PIDController<Radian, Velocity<Radian>>
  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetaAmpkP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetaAmpkI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )
  var desiredAngle = 0.0.degrees
  private val sizeOfMedianFilter =
    LoggedTunableNumber("TargetSpeakerCommand/sizeOfMedianFilter", 10.0)
  var angleMedianFilter = MedianFilter(10)

  init {
    addRequirements(drivetrain)

    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    if (!(RobotBase.isSimulation())) {

      thetakP.initDefault(DrivetrainConstants.PID.TELEOP_ALIGN_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.TELEOP_ALIGN_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.TELEOP_ALIGN_PID_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_ALIGN_PID_KP,
          DrivetrainConstants.PID.TELEOP_ALIGN_PID_KI,
          DrivetrainConstants.PID.TELEOP_ALIGN_PID_KD
        )
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)

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
    thetaPID.reset() // maybe do first for x?
    angleMedianFilter.reset()
    /*
    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    }

     */
  }

  override fun execute() {
    if (sizeOfMedianFilter.hasChanged()) {
      angleMedianFilter = MedianFilter(sizeOfMedianFilter.get().toInt())
    }

    drivetrain.defaultCommand.end(true)
    Logger.recordOutput("ActiveCommands/TargetAngleCommand", true)
    Logger.recordOutput(
      "Testing/CurrentDrivetrainRotation", drivetrain.odomTRobot.rotation.inDegrees
    )

    val odomTRobot = drivetrain.odomTRobot
    val odomTSpeaker = drivetrain.odomTSpeaker
    val robotTSpeaker = odomTRobot.inverse().transformBy(odomTSpeaker)

    desiredAngle =
      atan2(
        robotTSpeaker.y.inMeters -
          (
            drivetrain.robotVelocity.y *
              robotTSpeaker.translation.magnitude.absoluteValue / 7
            )
            .value,
        robotTSpeaker.x.inMeters -
          10.inches.inMeters -
          (
            drivetrain.robotVelocity.x *
              robotTSpeaker.translation.magnitude.absoluteValue / 7
            )
            .value
      )
        .radians
    val filteredDesiredAngle = angleMedianFilter.calculate(desiredAngle.inDegrees)

    val thetaFeedback =
      thetaPID.calculate(odomTRobot.rotation, odomTRobot.rotation + filteredDesiredAngle.degrees)

    Logger.recordOutput("TargetSpeakerCommand/desiredAngleInDegrees", desiredAngle.inDegrees)
    Logger.recordOutput("TargetSpeakerCommand/filteredDesiredAngleInDegrees", filteredDesiredAngle)
    Logger.recordOutput("TargetSpeakerCommand/errorInDegrees", thetaPID.error.inDegrees)
    Logger.recordOutput("TargetSpeakerCommand/thetaFeedbackInDPS", thetaFeedback.inDegreesPerSecond)
    Logger.recordOutput("TargetSpeakerCommand/relativeToRobotPose", robotTSpeaker.pose2d)

    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        thetaFeedback,
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    Logger.recordOutput("ActiveCommands/TargetAngleCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
