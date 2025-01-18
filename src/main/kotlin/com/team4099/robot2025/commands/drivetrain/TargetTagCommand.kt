package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.DrivetrainConstants.DRIVE_SETPOINT_MAX
import com.team4099.robot2025.config.constants.DrivetrainConstants.TURN_SETPOINT_MAX
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerDegree
import org.team4099.lib.units.derived.inMetersPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
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
  val limelight: LimelightVision,
  val targetAngle: () -> Angle,
) : Command() {

  private var thetaPID: PIDController<Radian, Velocity<Radian>>
  private var xPID: PIDController<Radian, Velocity<Meter>>

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

  val xkP =
    LoggedTunableValue(
      "Pathfollow/xkP",
      Pair({ it.inMetersPerSecondPerDegree }, { it.meters.perSecond.perDegree })
    )
  val xkI =
    LoggedTunableValue(
      "Pathfollow/xkI",
      Pair({ it.inMetersPerSecondPerDegreeSeconds }, { it.meters.perSecond.perDegreeSeconds })
    )
  val xkD =
    LoggedTunableValue(
      "Pathfollow/xkD",
      Pair(
        { it.inMetersPerSecondPerDegreePerSecond },
        { it.meters.perSecond.perDegreePerSecond }
      )
    )

  init {
    addRequirements(drivetrain)
    addRequirements(limelight)

    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    xPID = PIDController(xkP.get(), xkI.get(), xkD.get())

    if (!(RobotBase.isSimulation())) {

      thetakP.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KD
        )

      xkP.initDefault(DrivetrainConstants.PID.TELEOP_X_PID_KP)
      xkI.initDefault(DrivetrainConstants.PID.TELEOP_X_PID_KI)
      xkD.initDefault(DrivetrainConstants.PID.TELEOP_X_PID_KD)

      xPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_X_PID_KP,
          DrivetrainConstants.PID.TELEOP_X_PID_KI,
          DrivetrainConstants.PID.TELEOP_X_PID_KD
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

      xkP.initDefault(DrivetrainConstants.PID.SIM_TELEOP_X_PID_KP)
      xkI.initDefault(DrivetrainConstants.PID.SIM_TELEOP_X_PID_KI)
      xkD.initDefault(DrivetrainConstants.PID.SIM_TELEOP_X_PID_KD)

      xPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KD
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset() // maybe do first for x?
    xPID.reset()


    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    }

    if (xkP.hasChanged() || xkI.hasChanged() || xkD.hasChanged()) {
      xPID = PIDController(xkP.get(), xkI.get(), xkD.get())
    }


  }

  override fun execute() {

    drivetrain.defaultCommand.end(true)
    CustomLogger.recordDebugOutput("ActiveCommands/TargetTagCommand", true)
    Logger.recordOutput(
      "Testing/CurrentDrivetrainRotation", drivetrain.odomTRobot.rotation.inDegrees
    )

    var thetaFeedback = thetaPID.calculate(drivetrain.odomTRobot.rotation, targetAngle())

    CustomLogger.recordDebugOutput("Testing/thetaError", thetaPID.error.inDegrees)
    CustomLogger.recordDebugOutput("Testing/thetaFeedback", thetaFeedback.inDegreesPerSecond)


      drivetrain.currentRequest =
          Request.DrivetrainRequest.OpenLoop(
              thetaFeedback,
              driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
              fieldOriented = true
          )



    if (limelight.inputs.aprilTagTargets.size > 0 && thetaPID.error < 5.degrees) {

      var xFeedback = xPID.calculate(limelight.inputs.xAngle, 0.0.degrees)
      CustomLogger.recordDebugOutput("TagAlignment/xError", xPID.error.inDegrees)
      CustomLogger.recordDebugOutput("TagAlignment/xFeedback", xFeedback.inMetersPerSecond)

      val driveVector = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      var autoDriveVector =
        hypot(driveVector.first.inMetersPerSecond, driveVector.second.inMetersPerSecond)
      if (DriverStation.getAlliance().isPresent &&
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red
      ) {
        autoDriveVector =
          -hypot(driveVector.first.inMetersPerSecond, driveVector.second.inMetersPerSecond)
      }

      xFeedback = if (xFeedback > DRIVE_SETPOINT_MAX) DRIVE_SETPOINT_MAX else xFeedback
      xFeedback *= 0.25

      drivetrain.currentRequest =
        Request.DrivetrainRequest.OpenLoop(
          thetaFeedback,
          Pair(autoDriveVector.meters.perSecond, xFeedback),
          fieldOriented = false
        )
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/TargetTagCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
