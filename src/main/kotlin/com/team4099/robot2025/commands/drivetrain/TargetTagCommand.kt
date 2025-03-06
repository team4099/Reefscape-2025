package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.milli
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.hypot

class TargetTagCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val vision: Vision,
  val yTargetOffset: Length = 0.meters,
  val tagTargetID: Int = -1
) : Command() {

  private var thetaPID: PIDController<Radian, Velocity<Radian>>
  private var yPID: PIDController<Meter, Velocity<Meter>>
  private var xPID: PIDController<Meter, Velocity<Meter>>

  var timeStarted = Clock.fpgaTime

  var visionData = vision.lastTrigVisionUpdate

  val thetakP =
    LoggedTunableValue(
      "TagAlign/thetaAmpkP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "TagAlign/thetaAmpkI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "TagAlign/thetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  val ykP =
    LoggedTunableValue(
      "TagAlign/ykP", Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val ykI =
    LoggedTunableValue(
      "TagAlign/ykI",
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  val ykD =
    LoggedTunableValue(
      "TagAlign/ykD",
      Pair(
        { it.inMetersPerSecondPerMeterPerSecond }, { it.meters.perSecond.perMeterPerSecond }
      )
    )

  init {
    addRequirements(drivetrain)
    addRequirements(vision)

    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    yPID = PIDController(ykP.get(), ykI.get(), ykD.get())
    xPID = PIDController(
      ykP.get(),
      ykI.get(),
      ykD.get()
    )

    if (!(RobotBase.isSimulation())) {

      if (!DriverStation.isAutonomous()) {
        thetakP.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KP)
        thetakI.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KI)
        thetakD.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KD)

        thetaPID =
          PIDController(
            DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KD
          )
      }
      else {
        thetakP.initDefault(DrivetrainConstants.PID.AUTO_REEF_PID_KP)
        thetakI.initDefault(DrivetrainConstants.PID.AUTO_REEF_PID_KI)
        thetakD.initDefault(DrivetrainConstants.PID.AUTO_REEF_PID_KD)

        thetaPID =
          PIDController(
            DrivetrainConstants.PID.AUTO_REEF_PID_KP,
            DrivetrainConstants.PID.AUTO_REEF_PID_KI,
            DrivetrainConstants.PID.AUTO_REEF_PID_KD
          )
      }

      ykP.initDefault(DrivetrainConstants.PID.TELEOP_Y_PID_KP)
      ykI.initDefault(DrivetrainConstants.PID.TELEOP_Y_PID_KI)
      ykD.initDefault(DrivetrainConstants.PID.TELEOP_Y_PID_KD)

      yPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.TELEOP_Y_PID_KD
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

      ykP.initDefault(DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP)
      ykI.initDefault(DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI)
      ykD.initDefault(DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD)

      yPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  fun isAtSepoint(keepTrapping: Boolean = false): Boolean {
    val atSetPoint = if (!keepTrapping) {
      (thetaPID.error < 3.degrees && yPID.error < 2.inches && (vision.lastTrigVisionUpdate.robotTReefTag.translation.x.absoluteValue - 18.0.inches) < 3.inches)
    }
    else {
      (thetaPID.error < 3.degrees && yPID.error < 2.inches && (vision.lastTrigVisionUpdate.robotTReefTag.translation.x.absoluteValue - 18.0.inches) < 3.inches)
    }

    vision.isAligned = atSetPoint
    return atSetPoint && (Clock.fpgaTime - timeStarted > 50.0.milli.seconds)
  }

  override fun initialize() {


    thetaPID.reset() // maybe do first for x?
    yPID.reset()
    xPID.reset()

    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    }

    if (ykP.hasChanged() || ykI.hasChanged() || ykD.hasChanged()) {
      yPID = PIDController(ykP.get(), ykI.get(), ykD.get())
      xPID = PIDController(ykP.get(), ykI.get(), ykD.get())
    }

    if(tagTargetID != -1) {
      vision.currentRequest = Request.VisionRequest.TargetTag(arrayOf(tagTargetID))
    }

    vision.isAligned = true

    timeStarted = Clock.fpgaTime
  }

  override fun execute() {

    drivetrain.defaultCommand.end(true)
    CustomLogger.recordDebugOutput("ActiveCommands/TargetTagCommand", true)

    val visionData = vision.lastTrigVisionUpdate

    if (visionData.targetTagID != -1 &&
      visionData.robotTReefTag != Transform2d(Translation2d(0.meters, 0.meters), 0.degrees)
    ) {

      Logger.recordOutput("Testing/tagID", tagTargetID)

      val aprilTagAlignmentAngle =
        if (FMSData.isBlue) {
          VisionConstants.BLUE_REEF_TAG_THETA_ALIGNMENTS[tagTargetID]
        } else {
          VisionConstants.RED_REEF_TAG_THETA_ALIGNMENTS[tagTargetID]
        }

      var thetaFeedback =
        thetaPID.calculate(
          drivetrain.odomTRobot.rotation,
          aprilTagAlignmentAngle ?: drivetrain.odomTRobot.rotation
        )

      CustomLogger.recordDebugOutput(
        "Testing/CurrentDrivetrainRotation", drivetrain.odomTRobot.rotation.inDegrees
      )
      CustomLogger.recordOutput("TagAlignment/thetaError", thetaPID.error.inDegrees)
      CustomLogger.recordOutput("TagAlignment/thetaFeedback", thetaFeedback.inDegreesPerSecond)

      if (thetaPID.error.absoluteValue > 5.degrees) {
        drivetrain.currentRequest =
          Request.DrivetrainRequest.OpenLoop(
            thetaFeedback,
            driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
            fieldOriented = true
          )
      } else {

        var yFeedback = yPID.calculate(-visionData.robotTReefTag.translation.y, yTargetOffset - (visionData.robotTReefTag.translation.y + yTargetOffset) / 2.0)
        var xFeedBack = xPID.calculate(-visionData.robotTReefTag.translation.x, -(18.0).inches - (visionData.robotTReefTag.translation.x - 18.0.inches) / 1.75)


        CustomLogger.recordOutput("TagAlignment/yError", yPID.error.inMeters)
        CustomLogger.recordOutput(
          "TagAlignment/yFeedback",
          yFeedback.inMetersPerSecond,
        )

        CustomLogger.recordOutput("TagAlignment/xError", xPID.error.inMeters)
        CustomLogger.recordOutput(
          "TagAlignment/xFeedback",
          xFeedBack.inMetersPerSecond,
        )

        val driveVector = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)

        var autoDriveVector =
          hypot(driveVector.first.inMetersPerSecond, driveVector.second.inMetersPerSecond)

        drivetrain.currentRequest =
          Request.DrivetrainRequest.OpenLoop(
            thetaFeedback, Pair(xFeedBack, yFeedback), fieldOriented = false
          )
      }
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/TargetTagCommand", false)

    vision.currentRequest = Request.VisionRequest.TargetReef()

    vision.isAutoAligning = false
    vision.isAligned = false

    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
